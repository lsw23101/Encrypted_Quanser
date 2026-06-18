package main

import (
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"net"
	"time"

	utils "github.com/CDSL-EncryptedControl/CDSL/utils"
	RGSW "github.com/CDSL-EncryptedControl/CDSL/utils/core/RGSW"
	RLWE "github.com/CDSL-EncryptedControl/CDSL/utils/core/RLWE"
	"github.com/tuneinsight/lattigo/v6/core/rgsw"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
	"github.com/tuneinsight/lattigo/v6/ring"
)

// ============================================================
// Crypto parameters — identical to Lattigo/controller/offline.go
// ============================================================
const (
	encLogN    = 10
	encNTTFlag = true
	encS       = 1.0 / 1000.0    // controller gain quantization scale
	encL       = 1.0 / 1000000.0 // RLWE polynomial encoding scale
	encR       = 1.0 / 1000.0    // sensor input quantization scale
	encN       = 4               // state dimension
	encM       = 1               // input dimension
	encP       = 2               // output dimension
)

var encLogQ = []int{60}
var encLogP = []int{60}

// Controller matrices from offline.go
// F is already integer-valued; G, H, R are quantized by 1/s at runtime.
var Fmat = [][]float64{
	{0.0000, -0.0000, -0.0000, -0.0000},
	{1.0000, 0.0000, 0.0000, -2.0000},
	{-0.0000, 1.0000, 0.0000, 1.0000},
	{0.0000, -0.0000, 1.0000, 2.0000},
}
var Gmat = [][]float64{
	{1.0, -3.016},
	{0.0, -13.073},
	{0.0, -6.867},
	{0.0, 6.544},
}
var Hmat = [][]float64{
	{-18.810, 7.081, -39.702, -25.616},
}
var Rmat = [][]float64{
	{-0.025},
	{1.507},
	{0.769},
	{-0.729},
}

// scalRoundMat multiplies every element of m by scalar and rounds to nearest integer.
func scalRoundMat(scalar float64, m [][]float64) [][]float64 {
	result := make([][]float64, len(m))
	for i, row := range m {
		result[i] = make([]float64, len(row))
		for j, v := range row {
			result[i][j] = math.Round(scalar * v)
		}
	}
	return result
}

// writeFloats sends a float64 slice over TCP: [4-byte LE length | 8*n bytes LE doubles].
func writeFloats(conn net.Conn, values []float64) error {
	data := make([]byte, len(values)*8)
	for i, v := range values {
		binary.LittleEndian.PutUint64(data[i*8:], math.Float64bits(v))
	}
	lenBuf := make([]byte, 4)
	binary.LittleEndian.PutUint32(lenBuf, uint32(len(data)))
	if _, err := conn.Write(lenBuf); err != nil {
		return err
	}
	_, err := conn.Write(data)
	return err
}

// readFloats receives a float64 slice over TCP.
func readFloats(conn net.Conn, count int) ([]float64, error) {
	lenBuf := make([]byte, 4)
	if _, err := io.ReadFull(conn, lenBuf); err != nil {
		return nil, err
	}
	length := binary.LittleEndian.Uint32(lenBuf)
	data := make([]byte, length)
	if _, err := io.ReadFull(conn, data); err != nil {
		return nil, err
	}
	values := make([]float64, count)
	for i := range values {
		bits := binary.LittleEndian.Uint64(data[i*8:])
		values[i] = math.Float64frombits(bits)
	}
	return values, nil
}

func main() {
	fmt.Println("--- Encrypted Controller (Plaintext Transport) ---")

	// RLWE parameters
	params, err := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN:    encLogN,
		LogQ:    encLogQ,
		LogP:    encLogP,
		NTTFlag: encNTTFlag,
	})
	if err != nil {
		panic(err)
	}
	ringQ := params.RingQ()

	maxDim := math.Max(math.Max(float64(encN), float64(encM)), float64(encP))
	tau := int(math.Pow(2, math.Ceil(math.Log2(maxDim))))
	logTau := int(math.Log2(float64(tau)))

	// Galois elements for unpacking
	galEls := make([]uint64, logTau)
	for i := 0; i < logTau; i++ {
		galEls[i] = uint64(tau/int(math.Pow(2, float64(i))) + 1)
	}

	// Key generation
	fmt.Println(">> Generating keys...")
	kgen := rlwe.NewKeyGenerator(params)
	sk := kgen.GenSecretKeyNew()
	rlk := kgen.GenRelinearizationKeyNew(sk)
	gks := kgen.GenGaloisKeysNew(galEls, sk)

	encryptor := rlwe.NewEncryptor(params, sk)
	decryptor := rlwe.NewDecryptor(params, sk)
	encryptorRGSW := rgsw.NewEncryptor(params, sk)

	levelQ := params.QCount() - 1
	levelP := params.PCount() - 1

	// Monomials for RLWE unpacking
	monomials := make([]ring.Poly, logTau)
	for i := 0; i < logTau; i++ {
		monomials[i] = ringQ.NewPoly()
		idx := params.N() - params.N()/(1<<(i+1))
		monomials[i].Coeffs[0][idx] = 1
		ringQ.MForm(monomials[i], monomials[i])
		ringQ.NTT(monomials[i], monomials[i])
	}

	// Evaluators
	evkRGSW := rlwe.NewMemEvaluationKeySet(rlk)
	evkRLWE := rlwe.NewMemEvaluationKeySet(rlk, gks...)
	evaluatorRGSW := rgsw.NewEvaluator(params, evkRGSW)
	evaluatorRLWE := rlwe.NewEvaluator(params, evkRLWE)

	// Quantize and encrypt controller matrices as RGSW
	// F is used as-is (integer entries); G, H, R are scaled by 1/s
	fmt.Println(">> Encrypting controller matrices...")
	GBar := scalRoundMat(1/encS, Gmat)
	HBar := scalRoundMat(1/encS, Hmat)
	RBar := scalRoundMat(1/encS, Rmat)

	ctF := RGSW.EncPack(Fmat, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctG := RGSW.EncPack(GBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctH := RGSW.EncPack(HBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctR := RGSW.EncPack(RBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)

	// Initial state x = 0 → xBar = round(0 / (r*s)) = 0
	x0Bar := utils.RoundVec(utils.ScalVecMult(1/(encR*encS), []float64{0, 0, 0, 0}))
	xCtPackTemplate := RLWE.EncPack(x0Bar, tau, 1/encL, *encryptor, ringQ, params)

	fmt.Println("--- Ready ---")

	// TCP server
	HOST, PORT := "localhost", "8000"
	listener, err := net.Listen("tcp", HOST+":"+PORT)
	if err != nil {
		panic(err)
	}
	defer listener.Close()
	fmt.Printf("Waiting for Plant at %s:%s...\n", HOST, PORT)

	for {
		conn, err := listener.Accept()
		if err != nil {
			fmt.Println("Error accepting:", err)
			continue
		}
		fmt.Println(">> Plant Connected!")

		xCtPack := xCtPackTemplate.CopyNew()
		go handleEncCtrl(
			conn, xCtPack,
			ctF, ctG, ctH, ctR,
			evaluatorRGSW, evaluatorRLWE,
			encryptor, decryptor,
			ringQ, monomials, params, tau,
		)
	}
}

func handleEncCtrl(
	conn net.Conn,
	xCtPack *rlwe.Ciphertext,
	ctF, ctG, ctH, ctR []*rgsw.Ciphertext,
	evaluatorRGSW *rgsw.Evaluator,
	evaluatorRLWE *rlwe.Evaluator,
	encryptor *rlwe.Encryptor,
	decryptor *rlwe.Decryptor,
	ringQ *ring.Ring,
	monomials []ring.Poly,
	params rlwe.Parameters,
	tau int,
) {
	defer conn.Close()
	iter := 0

	fmt.Println(">> Controller Loop Started")

	// Scale applied when decrypting u = H*x:
	// u_coeff ≈ HBar * xBar * (1/L) = (H/s) * (x/(r*s)) * (1/L) = H*x / (r*s²*L)
	// → physical u = u_coeff * r*s²*L
	const uDecScale = encR * encS * encS * encL

	for {
		start := time.Now()

		// 1. Unpack x, compute encrypted u = H * x
		xCt := RLWE.UnpackCt(xCtPack, encN, tau, evaluatorRLWE, ringQ, monomials, params)
		uCtPack := RGSW.MultPack(xCt, ctH, evaluatorRGSW, ringQ, params)

		// 2. Decrypt u → physical value
		uVec := RLWE.DecUnpack(uCtPack, encM, tau, *decryptor, uDecScale, ringQ, params)
		u := uVec[0]

		// 3. Send plaintext u to Python plant
		if err := writeFloats(conn, []float64{u}); err != nil {
			fmt.Println("Write u Error:", err)
			return
		}

		// 4. Receive plaintext y from Python plant
		y, err := readFloats(conn, encP)
		if err != nil {
			if err != io.EOF {
				fmt.Println("Read y Error:", err)
			}
			return
		}

		// 5. Encrypt y freshly: yBar = round(y / r), then EncPack with scale 1/L
		yBar := utils.RoundVec(utils.ScalVecMult(1/encR, y))
		yCtPack := RLWE.EncPack(yBar, tau, 1/encL, *encryptor, ringQ, params)

		// 6. Re-encrypt u freshly (same scale as y): uBar = round(u / r)
		uBar := utils.RoundVec(utils.ScalVecMult(1/encR, []float64{u}))
		uCtPackReEnc := RLWE.EncPack(uBar, tau, 1/encL, *encryptor, ringQ, params)

		// 7. Unpack y and re-encrypted u for RGSW multiplication
		yCt := RLWE.UnpackCt(yCtPack, encP, tau, evaluatorRLWE, ringQ, monomials, params)
		uCt := RLWE.UnpackCt(uCtPackReEnc, encM, tau, evaluatorRLWE, ringQ, monomials, params)

		// 8. State update: x_new = F*x + G*y + R*u  (all encrypted RGSW mult)
		FxCt := RGSW.MultPack(xCt, ctF, evaluatorRGSW, ringQ, params)
		GyCt := RGSW.MultPack(yCt, ctG, evaluatorRGSW, ringQ, params)
		RuCt := RGSW.MultPack(uCt, ctR, evaluatorRGSW, ringQ, params)
		xCtPack = RLWE.Add(FxCt, GyCt, RuCt, params)

		elapsed := time.Since(start).Milliseconds()
		if iter%100 == 0 {
			fmt.Printf("Iter %d: u=%.4f  Cycle %d ms\n", iter, u, elapsed)
		}
		iter++
	}
}
