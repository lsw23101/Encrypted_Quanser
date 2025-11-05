// encrypted_controller_server.go
package main

import (
	"bufio"
	"fmt"
	"log"
	"math"
	"net"
	"strconv"
	"strings"
	"time"

	"github.com/CDSL-EncryptedControl/CDSL/utils"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
	"github.com/tuneinsight/lattigo/v6/ring"
	"github.com/tuneinsight/lattigo/v6/schemes/bgv"
)

// ======================= 사용자 선택 / 파라미터 =======================

// listen address for Python <-> Go
const addrListen = "127.0.0.1:9000"

// HE / quantization parameters
const (
	logN   = 12        // polynomial degree = 2^logN
	ptSize = uint64(27) // plaintext modulus ~ 2^ptSize (search)
	ctSize = 74       // total ciphertext modulus bits (split in two)
	r      = 0.001   // sensor quant scale
	s      = 0.001   // controller param scale
)

func main() {
	// ---------------- Encryption settings ----------------
	primeGen := ring.NewNTTFriendlyPrimesGenerator(ptSize, uint64(math.Pow(2, float64(logN)+1)))
	ptModulus, _ := primeGen.NextAlternatingPrime()
	logQ := []int{int(math.Floor(float64(ctSize) * 0.5)), int(math.Ceil(float64(ctSize) * 0.5))}

	params, err := bgv.NewParametersFromLiteral(bgv.ParametersLiteral{
		LogN:             logN,
		LogQ:             logQ,
		PlaintextModulus: ptModulus,
	})
	if err != nil {
		log.Fatalf("params: %v", err)
	}
	fmt.Println("Plaintext modulus:", params.PlaintextModulus())
	fmt.Println("Ciphertext modulus (Q):", params.QBigInt())
	fmt.Println("Ring degree (N):", params.N())
	fmt.Println("Scaling parameters 1/r:", 1/r, "1/s:", 1/s)

	// ---------------- Controller (IO form) ----------------
	// Hu, Hy are transpose of vecHu, vecHy from conversion.m
	Hu := [][]float64{
		{-0.0498, 0},
		{0.2513, 0},
		{-0.2869, 0},
		{0.2710, 0},
	}
	Hy := [][]float64{
		{-3.1796, 7.7947},
		{22.8571, -57.1848},
		{-50.4053, 132.1963},
		{31.5550, -92.4417},
	}

	// Dimensions: l = outputs, m = inputs, n = controller order, h = max(l,m)
	l := 2                   // dim(y) = 2
	m := 1                   // dim(u) = 1
	n := len(Hy)             // controller taps = 4
	h := int(math.Max(float64(l), float64(m))) // = 2

	// Secret key & primitives (built once)
	kgen := bgv.NewKeyGenerator(params)
	sk := kgen.GenSecretKeyNew()

	// Precompute controller parameters (encrypted once, reused)
	ctHy, ctHu := encControllerParams(Hy, Hu, params, sk)

	// Start TCP server
	ln, err := net.Listen("tcp", addrListen)
	if err != nil {
		log.Fatalf("listen(%s): %v", addrListen, err)
	}
	defer ln.Close()
	fmt.Println("[Go] Encrypted controller server listening on", addrListen)

	for {
		conn, err := ln.Accept()
		if err != nil {
			fmt.Println("[Go] accept error:", err)
			continue
		}
		if tcp, ok := conn.(*net.TCPConn); ok {
			tcp.SetNoDelay(true)
			tcp.SetKeepAlive(true)
		}
		go handleConn(conn, params, sk, ctHy, ctHu, n, m, l, h)
	}
}

// encControllerParams: Hy, Hu -> encrypt once (per row) as in the simulation code.
func encControllerParams(Hy, Hu [][]float64, params bgv.Parameters, sk *rlwe.SecretKey) (ctHy, ctHu []*rlwe.Ciphertext) {
	encoder := bgv.NewEncoder(params)
	encryptor := bgv.NewEncryptor(params, sk)

	n := len(Hy)
	ctHy = make([]*rlwe.Ciphertext, n)
	ctHu = make([]*rlwe.Ciphertext, n)

	for i := 0; i < n; i++ {
		// Hy row
		ptHy := bgv.NewPlaintext(params, params.MaxLevel())
		hyScaled := utils.ModVec(utils.RoundVec(utils.ScalVecMult(1/s, Hy[i])), params.PlaintextModulus())
		encoder.Encode(hyScaled, ptHy)
		ctHy[i], _ = encryptor.EncryptNew(ptHy)

		// Hu row
		ptHu := bgv.NewPlaintext(params, params.MaxLevel())
		huScaled := utils.ModVec(utils.RoundVec(utils.ScalVecMult(1/s, Hu[i])), params.PlaintextModulus())
		encoder.Encode(huScaled, ptHu)
		ctHu[i], _ = encryptor.EncryptNew(ptHu)
	}
	return
}

// Per-connection handler: maintains encrypted histories {ctY, ctU} and replies u for each incoming y.
func handleConn(
	conn net.Conn,
	params bgv.Parameters,
	sk *rlwe.SecretKey,
	ctHy, ctHu []*rlwe.Ciphertext,
	n, m, l, h int,
) {
	defer conn.Close()
	fmt.Println("[Go] New Python client from", conn.RemoteAddr())

	reader := bufio.NewReaderSize(conn, 64*1024)
	writer := bufio.NewWriterSize(conn, 64*1024)

	// Per-connection primitives (avoid cross-goroutine sharing)
	encoder := bgv.NewEncoder(params)
	encryptor := bgv.NewEncryptor(params, sk)
	decryptor := bgv.NewDecryptor(params, sk)
	eval := bgv.NewEvaluator(params, nil)
	bredparams := ring.GenBRedConstant(params.PlaintextModulus())

	// Initialize encrypted histories with zeros (same as yy0, uu0 in sim code)
	ctY := make([]*rlwe.Ciphertext, n)
	ctU := make([]*rlwe.Ciphertext, n)
	for i := 0; i < n; i++ {
		// zero y
		ptY := bgv.NewPlaintext(params, params.MaxLevel())
		encoder.Encode(utils.ModVec(utils.RoundVec(utils.ScalVecMult(1/r, utils.VecDuplicate(make([]float64, l), m, h))), params.PlaintextModulus()), ptY)
		ctY[i], _ = encryptor.EncryptNew(ptY)
		// zero u
		ptU := bgv.NewPlaintext(params, params.MaxLevel())
		encoder.Encode(utils.ModVec(utils.RoundVec(utils.ScalVecMult(1/r, utils.VecDuplicate(make([]float64, m), m, h))), params.PlaintextModulus()), ptU)
		ctU[i], _ = encryptor.EncryptNew(ptU)
	}

	var step int
	var last time.Time

	for {
		// -------- 1) TCP로 y=[theta,alpha] 수신 --------
		line, err := reader.ReadString('\n')
		if err != nil {
			fmt.Println("[Go] read error:", err)
			return
		}
		line = strings.TrimSpace(line)
		if line == "" {
			continue
		}
		parts := strings.SplitN(line, ",", 3)
		if len(parts) < 2 {
			fmt.Println("[Go] malformed line:", line)
			continue
		}
		theta, err0 := strconv.ParseFloat(strings.TrimSpace(parts[0]), 64)
		alpha, err1 := strconv.ParseFloat(strings.TrimSpace(parts[1]), 64)
		if err0 != nil || err1 != nil {
			fmt.Println("[Go] parse error:", err0, err1, "line:", line)
			continue
		}

		// 주기 로깅
		now := time.Now()
		if !last.IsZero() && step%50 == 0 {
			fmt.Printf("[Go] loop dt ≈ %.3f ms\n", now.Sub(last).Seconds()*1000)
		}
		last = now
		step++

		// -------- 2) y 암호화 (양자화/패킹) --------
		Y := []float64{theta, alpha}
		Ysens := utils.ModVec(
			utils.RoundVec(utils.ScalVecMult(1/r, utils.VecDuplicate(Y, m, h))),
			params.PlaintextModulus(),
		)
		ptYcur := bgv.NewPlaintext(params, params.MaxLevel())
		encoder.Encode(Ysens, ptYcur)
		Ycin, _ := encryptor.EncryptNew(ptYcur)

		// -------- 3) 암호화 제어 연산 Uout = sum(Hy_i*Y_{k-i} + Hu_i*U_{k-i}) --------
		Uout, _ := eval.MulNew(ctHy[0], ctY[0])
		eval.MulThenAdd(ctHu[0], ctU[0], Uout)
		for j := 1; j < n; j++ {
			eval.MulThenAdd(ctHy[j], ctY[j], Uout)
			eval.MulThenAdd(ctHu[j], ctU[j], Uout)
		}

		// -------- 4) 복호화하여 평문 u 생성 & 즉시 TCP 응답 --------
		Uact := make([]uint64, params.N()) // unpack buffer
		encoder.Decode(decryptor.DecryptNew(Uout), Uact)

		U := make([]float64, m)
		Usum := make([]uint64, m)
		for k := 0; k < m; k++ {
			// inner-sum over the h-pack
			Usum[k] = utils.VecSumUint(Uact[k*h:(k+1)*h], params.PlaintextModulus(), bredparams)
			U[k] = float64(r * s * utils.SignFloat(float64(Usum[k]), params.PlaintextModulus()))
		}

		// TCP 응답 (스칼라 한 줄)
		if _, err := fmt.Fprintf(writer, "%.6f\n", U[0]); err != nil {
			fmt.Println("[Go] write error:", err)
			return
		}
		if err := writer.Flush(); err != nil {
			fmt.Println("[Go] flush error:", err)
			return
		}

		// -------- 5) u 재-양자화/재-암호화 (다음 스텝을 위한 히스토리) --------
		ptUcur := bgv.NewPlaintext(params, params.MaxLevel())
		Udup := utils.ModVec(utils.RoundVec(utils.ScalVecMult(1/r, utils.VecDuplicate(U, m, h))), params.PlaintextModulus())
		encoder.Encode(Udup, ptUcur)
		Ucin, _ := encryptor.EncryptNew(ptUcur)

		// -------- 6) 상태(슬라이딩 윈도우) 업데이트 --------
		ctY = append(ctY[1:], Ycin)
		ctU = append(ctU[1:], Ucin)
	}
}
