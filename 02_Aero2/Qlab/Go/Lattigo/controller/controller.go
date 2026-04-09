package main

import (
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"net"
	"path/filepath"
	"time"

	// 1. 파일 유틸
	fileutils "github.com/CDSL-EncryptedControl/CDSL/Lattigo/utils"

	// 2. 라이브러리
	RGSW "github.com/CDSL-EncryptedControl/CDSL/utils/core/RGSW"
	RLWE "github.com/CDSL-EncryptedControl/CDSL/utils/core/RLWE"
	"github.com/tuneinsight/lattigo/v6/core/rgsw"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
	"github.com/tuneinsight/lattigo/v6/ring"
)

// ── Run parameters ──────────────────────────────────────────────────────────
const (
	FREQUENCY       = 50
	REF_PITCH_RAD   = 30.0 * math.Pi / 180.0
	REF_YAW_RAD     = -30.0 * math.Pi / 180.0
	REF_HALF_PERIOD = 10.0          // seconds per half-cycle (square wave)
	R_QUANT         = 1.0 / 1000.0 // y / reference quantization scale (== VAL_R)
	VAL_S           = 1.0 / 1000.0 // input scaling (== s in offline.go)
	VAL_L           = 1.0 / 1000000.0 // L = r*s combined scaling
)

// ── Plaintext matrices (N_bar, PBar) ────────────────────────────────────────
// N_bar 는 평문으로 처리: u = H*z_enc + N_bar*r
// NBar = round(N_bar / s) = round(N_bar × 1000)  [2×2]
var NBar = [][]float64{
	{64757, -52383},
	{53002, 62105},
}

// PBar = round(P_ / s) = round(P_ × 1000)  [4×2]
// offline.go 와 동일한 값 — state update 평문 항
var PBar = [][]float64{
	{2363, 576},
	{455, 3422},
	{-915, -220},
	{-174, -1292},
}

// ── TCP helpers ──────────────────────────────────────────────────────────────

func writeCiphertext(conn net.Conn, ct *rlwe.Ciphertext) error {
	data, err := ct.MarshalBinary()
	if err != nil {
		return err
	}
	lenBuf := make([]byte, 4)
	binary.LittleEndian.PutUint32(lenBuf, uint32(len(data)))
	if _, err := conn.Write(lenBuf); err != nil {
		return err
	}
	_, err = conn.Write(data)
	return err
}

func readCiphertext(conn net.Conn) (*rlwe.Ciphertext, error) {
	lenBuf := make([]byte, 4)
	if _, err := io.ReadFull(conn, lenBuf); err != nil {
		return nil, err
	}
	data := make([]byte, binary.LittleEndian.Uint32(lenBuf))
	if _, err := io.ReadFull(conn, data); err != nil {
		return nil, err
	}
	ct := new(rlwe.Ciphertext)
	if err := ct.UnmarshalBinary(data); err != nil {
		return nil, err
	}
	return ct, nil
}

// ── Math helpers ─────────────────────────────────────────────────────────────

// matVec: 행렬-벡터 곱 (평문)
func matVec(mat [][]float64, vec []float64) []float64 {
	result := make([]float64, len(mat))
	for i, row := range mat {
		for j, v := range row {
			result[i] += v * vec[j]
		}
	}
	return result
}

// encodePlainPack: 정수 벡터를 CDSL 팩킹 형식의 ring.Poly 로 인코딩
// 반환된 Poly 는 NTT 변환 완료 → ringQ.Add(ct.Value[0], poly, ct.Value[0]) 로 직접 가산.
// 패킹 규칙: 원소 i → 계수 위치 i*(N/tau)  (CDSL EncPack 과 동일)
func encodePlainPack(vec []float64, tau int, params rlwe.Parameters, ringQ *ring.Ring) ring.Poly {
	N    := params.N()
	step := N / tau
	Q    := ringQ.ModuliChain()[0]

	poly := ring.NewPoly(N, params.MaxLevel())

	for i, v := range vec {
		intVal := int64(math.Round(v))
		var coeff uint64
		if intVal >= 0 {
			coeff = uint64(intVal) % Q
		} else {
			coeff = Q - uint64(-intVal)%Q
		}
		poly.Coeffs[0][i*step] = coeff
	}

	// 암호문은 NTTFlag=true → 평문도 NTT 도메인으로 변환
	ringQ.NTT(poly, poly)
	return poly
}

// ── Main ─────────────────────────────────────────────────────────────────────

func main() {
	fmt.Println("--- RLWE params (Aero2) ---")
	loadDir := "enc_data"

	// RLWE 파라미터
	params, _ := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN: 12, LogQ: []int{60}, LogP: []int{60}, NTTFlag: true,
	})
	ringQ := params.RingQ()

	// 차원: state n=4, input m=2, output p=2
	n, m, p := 4, 2, 2

	maxDim := math.Max(math.Max(float64(n), float64(m)), float64(p))
	tau := int(math.Pow(2, math.Ceil(math.Log2(maxDim))))

	fmt.Printf("n=%d, m=%d, p=%d, tau=%d\n", n, m, p, tau)
	fmt.Println("Degree of polynomials:", params.N())

	// Unpack 용 Monomial 생성
	logn := int(math.Log2(float64(tau)))
	monomials := make([]ring.Poly, logn)
	for i := 0; i < logn; i++ {
		monomials[i] = ringQ.NewPoly()
		idx := params.N() - params.N()/(1<<(i+1))
		monomials[i].Coeffs[0][idx] = 1
		ringQ.MForm(monomials[i], monomials[i])
		ringQ.NTT(monomials[i], monomials[i])
	}

	// 키 로드 (비밀키 제외)
	rlk := new(rlwe.RelinearizationKey)
	if err := fileutils.ReadRT(filepath.Join(loadDir, "rlk.dat"), rlk); err != nil {
		panic(err)
	}
	gks, err := fileutils.LoadGaloisKeys(loadDir)
	if err != nil {
		panic(err)
	}

	// 암호화된 행렬 로드
	fmt.Println(">> Loading encrypted matrices...")
	ctF, _ := fileutils.LoadRGSWPack(loadDir, "ctF")
	ctG, _ := fileutils.LoadRGSWPack(loadDir, "ctG")
	ctH, _ := fileutils.LoadRGSWPack(loadDir, "ctH")
	ctR, _ := fileutils.LoadRGSWPack(loadDir, "ctR")
	ctP, _ := fileutils.LoadRGSWPack(loadDir, "ctP")

	// 초기 상태 로드
	xCtPackTemplate := new(rlwe.Ciphertext)
	if err := fileutils.ReadRT(filepath.Join(loadDir, "xCtPack.dat"), xCtPackTemplate); err != nil {
		panic(err)
	}

	// Evaluator 구성
	evkRGSW := rlwe.NewMemEvaluationKeySet(rlk)
	evkRLWE := rlwe.NewMemEvaluationKeySet(rlk, gks...)
	evaluatorRGSW := rgsw.NewEvaluator(params, evkRGSW)
	evaluatorRLWE := rlwe.NewEvaluator(params, evkRLWE)

	fmt.Println("--- Ready ---")

	// TCP 서버 시작
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
		go handleEncryptedControl(
			conn, xCtPack,
			ctF, ctG, ctH, ctR, ctP,
			evaluatorRGSW, evaluatorRLWE,
			ringQ, monomials, params,
			n, m, p, tau,
		)
	}
}

// ── Encrypted control loop ───────────────────────────────────────────────────

func handleEncryptedControl(
	conn net.Conn,
	xCtPack *rlwe.Ciphertext,
	ctF, ctG, ctH, ctR, ctP []*rgsw.Ciphertext,
	evaluatorRGSW *rgsw.Evaluator,
	evaluatorRLWE *rlwe.Evaluator,
	ringQ *ring.Ring,
	monomials []ring.Poly,
	params rlwe.Parameters,
	n, m, p, tau int,
) {
	defer conn.Close()
	iter := 0

	fmt.Println(">> Controller Loop Started")

	// 통신 흐름 (매 스텝):
	//   [1] r 계산 (plaintext)
	//   [2] u_enc = H*x_enc + N_bar*r → send u_enc
	//   [3] recv combinedCt = pack([y_enc | u_reenc])
	//   [4] x_enc = F*x_enc + G*y_enc + R*u_reenc + P*r

	for {
		start := time.Now()

		// [1] 사각파 레퍼런스 (10초 주기, plaintext)
		halfCycle := int(float64(iter) / (REF_HALF_PERIOD * FREQUENCY))
		refSign   := 1.0 - 2.0*float64(halfCycle%2)
		// refBar = round(ref / R_QUANT): G, R, H, P 와 동일한 정수 스케일
		refBar := []float64{
			math.Round(refSign * REF_PITCH_RAD / R_QUANT),
			math.Round(refSign * REF_YAW_RAD   / R_QUANT),
		}

		// [2a] 암호 연산: H * x_enc
		xCt     := RLWE.UnpackCt(xCtPack, n, tau, evaluatorRLWE, ringQ, monomials, params)
		uCtPack := RGSW.MultPack(xCt, ctH, evaluatorRGSW, ringQ, params)

		// [2b] 평문 연산: N_bar * r → 평문 팩킹 후 암호문 c0 에 직접 가산
		//      u_enc = H*x_enc + N_bar*r
		//
		//      uCtPack 계수 단위 = value / (r*s²*L)
		//      nBarR_raw = NBar * refBar = (N_bar/s) * (ref/r) = N_bar*ref / (r*s)
		//      → 목표 계수 = N_bar*ref / (r*s²*L) → 추가로 1/(s*L) = 1e9 배 필요
		nBarR_raw := matVec(NBar, refBar)
		scaleU    := 1.0 / (VAL_S * VAL_L) // 1e9
		nBarR     := make([]float64, len(nBarR_raw))
		for i, v := range nBarR_raw { nBarR[i] = v * scaleU }
		ptNbarR := encodePlainPack(nBarR, tau, params, ringQ)  // ring.Poly, NTT 도메인
		ringQ.Add(uCtPack.Value[0], ptNbarR, uCtPack.Value[0])

		// [2c] send u_enc → Plant
		if err := writeCiphertext(conn, uCtPack); err != nil {
			fmt.Println("Write u Error:", err)
			return
		}

		// [3] recv pack([y_enc | u_reenc])  — p+m = 4 개 packed
		combinedCt, err := readCiphertext(conn)
		if err != nil {
			if err != io.EOF {
				fmt.Println("Read Combined CT Error:", err)
			}
			return
		}

		// unpack → [y_0, y_1, u_reenc_0, u_reenc_1]
		totalUnpacked  := RLWE.UnpackCt(combinedCt, p+m, tau, evaluatorRLWE, ringQ, monomials, params)
		yCt            := totalUnpacked[:p]
		uReEncUnpacked := totalUnpacked[p : p+m]

		// [4a] 암호 연산: F*x + G*y + R*u_reenc
		FxCt    := RGSW.MultPack(xCt,            ctF, evaluatorRGSW, ringQ, params)
		GyCt    := RGSW.MultPack(yCt,            ctG, evaluatorRGSW, ringQ, params)
		RuCt    := RGSW.MultPack(uReEncUnpacked, ctR, evaluatorRGSW, ringQ, params)
		xCtPack  = RLWE.Add(FxCt, GyCt, RuCt, params)

		// [4b] 평문 연산: P * r → 평문 팩킹 후 암호문 c0 에 직접 가산
		//      x_enc = (F*x + G*y + R*u) + P*r
		//
		//      xCtPack 계수 단위 = value / (r*s*L)
		//      pBarR_raw = PBar * refBar = (P/s) * (ref/r) = P*ref / (r*s)
		//      → 목표 계수 = P*ref / (r*s*L) → 추가로 1/L = 1e6 배 필요
		pBarR_raw := matVec(PBar, refBar)
		scaleX    := 1.0 / VAL_L // 1e6
		pBarR     := make([]float64, len(pBarR_raw))
		for i, v := range pBarR_raw { pBarR[i] = v * scaleX }
		ptPbarR := encodePlainPack(pBarR, tau, params, ringQ)  // ring.Poly, NTT 도메인
		ringQ.Add(xCtPack.Value[0], ptPbarR, xCtPack.Value[0])

		elapsed := time.Since(start).Milliseconds()
		if iter%100 == 0 {
			fmt.Printf("Iter %d: Cycle Time %d ms\n", iter, elapsed)
		}
		iter++
	}
}
