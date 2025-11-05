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

	utils "github.com/CDSL-EncryptedControl/CDSL/utils"
	RGSW "github.com/CDSL-EncryptedControl/CDSL/utils/core/RGSW"
	RLWE "github.com/CDSL-EncryptedControl/CDSL/utils/core/RLWE"

	"github.com/tuneinsight/lattigo/v6/core/rgsw"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
	"github.com/tuneinsight/lattigo/v6/ring"
)

// ======================= 사용자 선택 / 파라미터 =======================

// listen address for Python <-> Go
const addrListen = "127.0.0.1:9000"

// Quantization / scaling parameters (시뮬레이션과 동일)
const (
	s = 1.0 / 10000.0 // controller param scale
	L = 1.0 / 10000.0  // inner packing scale
	r = 1.0 / 10000.0 // sensor/output scale
)

func main() {
	// ---------------- Encryption settings (RLWE/RGSW) ----------------
	params, err := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN:    12,        // poly degree = 4096
		LogQ:    []int{56}, // ciphertext modulus Q
		LogP:    []int{56}, // special modulus P
		NTTFlag: true,
	})
	if err != nil {
		log.Fatalf("rlwe params: %v", err)
	}
	fmt.Println("Degree (N):", params.N())
	fmt.Println("Ciphertext modulus Q:", params.QBigInt())
	fmt.Println("Special modulus   P:", params.PBigInt())
	fmt.Println("Secret key dist:", params.Xs())
	fmt.Println("Error  dist:", params.Xe())
	fmt.Printf("Scaling parameters 1/L: %v, 1/s: %v, 1/r: %v\n", 1/L, 1/s, 1/r)

	// ---------------- Controller (state-space, IO form) ----------------
	// 차원: n=state(4), m=inputs(u)=1, p=outputs(y)=2

	// ```
	// 바꿔줄 부분 !!
	// ```
	F := [][]float64{
		{ -0, 0, -0, 0 },
		{ 1, -0, 0, -2 },
		{ 0, 1, -0, 1 },
		{ -0, 0, 1, 2 },
	}

	G := [][]float64{
		{ 1.0000, -2.0645 },
		{ -0.0000, -5.5302 },
		{ 0.0000, -2.7987 },
		{ 0.0000, 2.7513 },
	}

	H := [][]float64{
		{ 73.8783, 29.7889, 185.7358, 246.4416 },
	}

	R := [][]float64{
		{ 0.0035 },
		{ -0.4396 },
		{ -0.2204 },
		{ 0.2132 },
	}

	// 초기 상태 (암호 컨트롤러 내부 state)
	x_ini := []float64{-1, 0, -1, 0}

	// 차원
	n := len(F)    // 4
	m := len(H)    // 1 (u scalar)
	p := len(G[0]) // 2 (y=[theta,alpha])

	// ---------------- Pack/Unpack 준비 ----------------
	levelQ := params.QCount() - 1
	levelP := params.PCount() - 1
	ringQ := params.RingQ()

	// tau = pow2 >= max(n,m,p)
	maxDim := math.Max(math.Max(float64(n), float64(m)), float64(p))
	tau := int(math.Pow(2, math.Ceil(math.Log2(maxDim))))

	// Unpack용 모노미얼
	logTau := int(math.Log2(float64(tau)))
	monomials := make([]ring.Poly, logTau)
	for i := 0; i < logTau; i++ {
		monomials[i] = ringQ.NewPoly()
		idx := params.N() - params.N()/(1<<(i+1))
		monomials[i].Coeffs[0][idx] = 1
		ringQ.MForm(monomials[i], monomials[i])
		ringQ.NTT(monomials[i], monomials[i])
	}
	// Rotations (unpack)
	galEls := make([]uint64, logTau)
	for i := 0; i < logTau; i++ {
		galEls[i] = uint64(tau/int(math.Pow(2, float64(i))) + 1)
	}

	// ---------------- 키/평가자 준비 (오프라인) ----------------
	kgen := rlwe.NewKeyGenerator(params)
	sk := kgen.GenSecretKeyNew()
	rlk := kgen.GenRelinearizationKeyNew(sk)
	evkRGSW := rlwe.NewMemEvaluationKeySet(rlk)
	evkRLWE := rlwe.NewMemEvaluationKeySet(rlk, kgen.GenGaloisKeysNew(galEls, sk)...)

	encryptorRGSW := rgsw.NewEncryptor(params, sk)

	// 컨트롤러 계수 양자화 후 RGSW 암호화 (오프라인, 1회)
	GBar := utils.ScalMatMult(1/s, G)
	HBar := utils.ScalMatMult(1/s, H)
	RBar := utils.ScalMatMult(1/s, R)

	ctF := RGSW.EncPack(F, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctG := RGSW.EncPack(GBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctH := RGSW.EncPack(HBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)
	ctR := RGSW.EncPack(RBar, tau, encryptorRGSW, levelQ, levelP, ringQ, params)

	// 상태 초기화용 (각 연결에서 사용)
	xBar := utils.RoundVec(utils.ScalVecMult(1/(r*s), x_ini))

	// ---------------- TCP 서버 시작 ----------------
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

		// 연결마다 고루틴 실행 (통신 포맷 유지)
		go func(c net.Conn) {
			defer c.Close()
			fmt.Println("[Go] New Python client from", c.RemoteAddr())

			reader := bufio.NewReaderSize(c, 64*1024)
			writer := bufio.NewWriterSize(c, 64*1024)

			// 연결 전용 암호 객체
			encryptorRLWE := rlwe.NewEncryptor(params, sk)
			decryptorRLWE := rlwe.NewDecryptor(params, sk)
			evaluatorRGSW := rgsw.NewEvaluator(params, evkRGSW)
			evaluatorRLWE := rlwe.NewEvaluator(params, evkRLWE)

			// 암호 상태 x 초기화 (pack)
			xCtPack := RLWE.EncPack(xBar, tau, 1/L, *encryptorRLWE, ringQ, params)

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

				// === 타이밍 시작점: y를 정상 파싱한 즉시 ===
				t0 := time.Now()

				// -------- 2) y 암호화 (양자화/패킹) --------
				Y := []float64{theta, alpha} // p=2
				yBar := utils.RoundVec(utils.ScalVecMult(1/r, Y))
				yCtPack := RLWE.EncPack(yBar, tau, 1/L, *encryptorRLWE, ringQ, params)

				t1 := time.Now() // enc 끝

				// -------- 3) 암호화 제어 연산: u = H x --------
				// 상태/입력 언팩
				xCt := RLWE.UnpackCt(xCtPack, n, tau, evaluatorRLWE, ringQ, monomials, params) // n=4
				yCt := RLWE.UnpackCt(yCtPack, p, tau, evaluatorRLWE, ringQ, monomials, params) // p=2

				// 출력 u = H x (pack 형태)
				uCtPack := RGSW.MultPack(xCt, ctH, evaluatorRGSW, ringQ, params)

				t2 := time.Now() // eval(u) 끝

				// -------- 4) 복호화하여 평문 u 생성 --------
				// DecUnpack(scale=r*s*s*L) → 실수 복원
				u := RLWE.DecUnpack(uCtPack, m, tau, *decryptorRLWE, r*s*s*L, ringQ, params) // m=1

				t3 := time.Now() // dec 끝

				// -------- 4.5) TCP 응답 (스칼라 한 줄) --------
				if _, err := fmt.Fprintf(writer, "%.6f\n", u[0]); err != nil {
					fmt.Println("[Go] write error:", err)
					return
				}
				if err := writer.Flush(); err != nil {
					fmt.Println("[Go] flush error:", err)
					return
				}

				t4 := time.Now() // flush까지 포함한 end-to-end

				// -------- 5) u 재-암호화 (다음 스텝을 위한 상태업데이트용) --------
				uBar := utils.RoundVec(utils.ScalVecMult(1/r, u)) // len=1
				uReEnc := RLWE.Enc(uBar, 1/L, *encryptorRLWE, ringQ, params)

				// -------- 6) 상태업데이트: x <- F x + G y + R u --------
				FxCt := RGSW.MultPack(xCt, ctF, evaluatorRGSW, ringQ, params)
				GyCt := RGSW.MultPack(yCt, ctG, evaluatorRGSW, ringQ, params)
				RuCt := RGSW.MultPack(uReEnc, ctR, evaluatorRGSW, ringQ, params)
				xCtPack = RLWE.Add(FxCt, GyCt, RuCt, params)

				// -------- 7) 지연 로깅 (통신 로그 형식 유지) --------
				enc := t1.Sub(t0)
				evald := t2.Sub(t1) // u 계산만 (상태업데이트 시간은 로그에 포함하지 않음)
				dec := t3.Sub(t2)
				io := t4.Sub(t3)
				total := t4.Sub(t0)

				fmt.Printf("[Go] e2e latency: %.3f ms (enc %.3f | eval %.3f | dec %.3f | io %.3f) \n",
					float64(total.Microseconds())/1000.0,
					float64(enc.Microseconds())/1000.0,
					float64(evald.Microseconds())/1000.0,
					float64(dec.Microseconds())/1000.0,
					float64(io.Microseconds())/1000.0,
				)
			}
		}(conn)
	}
}
