package main

// 파이썬(y 수신↔u 송신) 서버 + 암호 컨트롤러(클라) 동시 동작
// - 파이썬에서 theta,alpha 라인 수신 → 로컬 관측/제어(F,G,H)로 u 계산하여 회신
// - 동시에 동일 y를 암호화(Ycin) → 컨트롤러에 송신 → Uout 수신/복호/재암호화(Ucin) → 송신(ACK 대기)

import (
	"Quanser_Enc_test/com_utils"
	"bufio"
	"fmt"
	"log"
	"math"
	"net"
	"os"
	"strconv"
	"strings"
	"time"

	"github.com/CDSL-EncryptedControl/CDSL/utils"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
	"github.com/tuneinsight/lattigo/v6/ring"
	"github.com/tuneinsight/lattigo/v6/schemes/bgv"
)

/************* 로컬 상태공간/게인 (파이썬과 동일) *************/
var F = [4][4]float64{
	{0.0620, -0.3381, 0.0318, -0.0202},
	{-0.0258, -0.3867, 0.0117, 0.0001},
	{-7.8511, -27.7120, 2.1869, -2.0175},
	{2.4549, -41.8777, 1.1819, -0.9864},
}
var G = [4][2]float64{
	{0.9798, 0.1224},
	{0.0673, 1.1956},
	{12.0512, 6.0503},
	{1.7277, 22.6045},
}
var H = [4]float64{4.1822, -24.5940, 1.1820, -2.0389}

/************* 주소/스케일 *************/
const (
	addrPythonServer    = "127.0.0.1:9000" // 파이썬이 접속해오는 서버(Go)
	addrEncController   = "127.0.0.1:8080" // 암호 컨트롤러(Go가 클라이언트로 접속)
	alphaResetDegAbs    = 30.0             // 큰 각도에서 관측기 리셋
	fixedCiphertextSize = 196966           // 기존 프로토콜 유지 (고정 길이)
)

func main() {
	/************* BGV 파라미터/키 *************/
	logN := 12
	ptSize := uint64(28) // log2(plaintext modulus)
	ctSize := int(74)    // log2(ciphertext modulus chain product)

	// 스케일 (기존 코드 유지)
	r := 0.00020 // Y 스케일
	s := 0.00010 // U 스케일
	fmt.Println("Scaling 1/r:", 1/r, "  1/s:", 1/s)

	primeGen := ring.NewNTTFriendlyPrimesGenerator(ptSize, uint64(math.Pow(2, float64(logN)+1)))
	ptModulus, _ := primeGen.NextAlternatingPrime()
	fmt.Println("Plaintext modulus:", ptModulus)

	logQ := []int{int(math.Floor(float64(ctSize) * 0.5)), int(math.Ceil(float64(ctSize) * 0.5))}
	params, err := bgv.NewParametersFromLiteral(bgv.ParametersLiteral{
		LogN:             logN,
		LogQ:             logQ,
		PlaintextModulus: ptModulus,
	})
	if err != nil {
		fmt.Println("BGV 파라미터 생성 실패:", err)
		return
	}
	fmt.Println("Ciphertext modulus:", params.QBigInt())
	fmt.Println("Degree of polynomials:", params.N())

	// sk 로드
	sk := rlwe.NewSecretKey(params)
	if err := com_utils.ReadFromFile("sk.dat", sk); err != nil {
		fmt.Println("비밀키(sk.dat) 로드 실패:", err)
		return
	}
	encryptor := bgv.NewEncryptor(params, sk)
	decryptor := bgv.NewDecryptor(params, sk)
	encoder := bgv.NewEncoder(params)
	bredparams := ring.GenBRedConstant(params.PlaintextModulus())

	/************* 암호 컨트롤러에 클라이언트로 접속 *************/
	encConn, err := net.Dial("tcp", addrEncController)
	if err != nil {
		fmt.Println("암호 컨트롤러 접속 실패:", err)
		os.Exit(1)
	}
	defer encConn.Close()
	if tcp, ok := encConn.(*net.TCPConn); ok {
		tcp.SetNoDelay(true)
		tcp.SetKeepAlive(true)
	}
	encR := bufio.NewReaderSize(encConn, 256*1024)
	encW := bufio.NewWriterSize(encConn, 256*1024)
	fmt.Println("[ENC] Connected to controller at", addrEncController)

	/************* 파이썬에서 접속해오는 서버 오픈 *************/
	ln, err := net.Listen("tcp", addrPythonServer)
	if err != nil {
		log.Fatalf("[Go] listen error: %v", err)
	}
	defer ln.Close()
	fmt.Println("[Go] Python-facing server listening on", addrPythonServer)

	for {
		pyConn, err := ln.Accept()
		if err != nil {
			fmt.Println("[Go] accept error:", err)
			continue
		}
		if tcp, ok := pyConn.(*net.TCPConn); ok {
			tcp.SetNoDelay(true)
			tcp.SetKeepAlive(true)
		}
		go handlePythonConn(pyConn, encryptor, decryptor, encoder, params, bredparams, r, s, encR, encW)
	}
}

func handlePythonConn(
	pyConn net.Conn,
	encryptor rlwe.Encryptor,
	decryptor rlwe.Decryptor,
	encoder *bgv.Encoder,
	params bgv.Parameters,
	bredparams uint64,
	r, s float64,
	encR *bufio.Reader,
	encW *bufio.Writer,
) {
	defer pyConn.Close()
	fmt.Println("[Go] New Python client:", pyConn.RemoteAddr())

	pyR := bufio.NewReaderSize(pyConn, 64*1024)
	pyW := bufio.NewWriterSize(pyConn, 64*1024)

	// 로컬 관측기 상태
	var xhat [4]float64
	var nLines int
	last := time.Time{}

	// 루프: 파이썬이 y를 보내줄 때마다 1회 처리
	for {
		startLoop := time.Now()

		// ---- 1) 파이썬 → y=[theta,alpha] 수신 ----
		line, err := pyR.ReadString('\n')
		if err != nil {
			fmt.Println("[Go] python read error:", err)
			return
		}
		parts := strings.Split(strings.TrimSpace(line), ",")
		if len(parts) < 2 {
			fmt.Println("[Go] malformed line:", line)
			continue
		}
		theta, err0 := strconv.ParseFloat(strings.TrimSpace(parts[0]), 64)
		alpha, err1 := strconv.ParseFloat(strings.TrimSpace(parts[1]), 64)
		if err0 != nil || err1 != nil {
			fmt.Println("[Go] parse error:", err0, err1, " line:", line)
			continue
		}

		// 주기 모니터
		now := time.Now()
		if !last.IsZero() && nLines%50 == 0 {
			fmt.Printf("[Go] loop dt ≈ %.3f ms\n", now.Sub(last).Seconds()*1000)
		}
		last = now
		nLines++

		// ---- 2) 로컬 제어 u = -H xhat (옵저버: xhat=F xhat + G y) ----
		alphaDeg := math.Abs(alpha * 180.0 / math.Pi)
		if alphaDeg > alphaResetDegAbs {
			for i := range xhat {
				xhat[i] = 0.0
			}
		}
		u := 0.0
		for i := 0; i < 4; i++ {
			u += H[i] * xhat[i]
		}
		u = -u

		// xhat 업데이트
		var xnext [4]float64
		for i := 0; i < 4; i++ {
			sum := 0.0
			for j := 0; j < 4; j++ {
				sum += F[i][j] * xhat[j]
			}
			sum += G[i][0]*theta + G[i][1]*alpha
			xnext[i] = sum
		}
		xhat = xnext

		// ---- 3) (파이썬 회신) 로컬 u 송신 ----
		if _, err := fmt.Fprintf(pyW, "%.6f\n", u); err != nil {
			fmt.Println("[Go] python write error:", err)
			return
		}
		if err := pyW.Flush(); err != nil {
			fmt.Println("[Go] python flush error:", err)
			return
		}

		// ---- 4) (암호 경로) y를 암호화해 컨트롤러에 송신 ----
		Y := []float64{theta, alpha}
		l, m := 2, 1
		h := int(math.Max(float64(l), float64(m)))

		Ysens := utils.ModVec(
			utils.RoundVec(
				utils.ScalVecMult(1/r, utils.VecDuplicate(Y, m, h)),
			),
			params.PlaintextModulus(),
		)
		Ypacked := bgv.NewPlaintext(params, params.MaxLevel())
		encoder.Encode(Ysens, Ypacked)
		Ycin, _ := encryptor.EncryptNew(Ypacked)

		serializedY, err := Ycin.MarshalBinary()
		if err != nil {
			fmt.Println("[ENC] Ycin marshal error:", err)
			return
		}
		if _, err = encW.Write(serializedY); err != nil {
			fmt.Println("[ENC] Ycin write error:", err)
			return
		}
		if err := encW.Flush(); err != nil {
			fmt.Println("[ENC] Ycin flush error:", err)
			return
		}
		fmt.Println("[ENC] Ycin sent. t=", time.Since(startLoop))

		// ---- 5) (암호 경로) Uout(암호) 수신 → 복호/복원 ----
		totalData, err := com_utils.ReadFullData(encR, fixedCiphertextSize)
		if err != nil {
			fmt.Println("[ENC] Uout read error:", err)
			return
		}
		Uout := rlwe.NewCiphertext(params, params.MaxLevel())
		if err = Uout.UnmarshalBinary(totalData[:fixedCiphertextSize]); err != nil {
			fmt.Println("[ENC] Uout unmarshal error:", err)
			return
		}
		Uact := make([]uint64, params.N())
		encoder.Decode(decryptor.DecryptNew(Uout), Uact)

		// 블록 합(inner sum) + 부호복원 + 스케일 복원 (기존 코드식 유지: r*s 곱)
		U := make([]float64, m)
		for k := 0; k < m; k++ {
			usum := utils.VecSumUint(Uact[k*h:(k+1)*h], params.PlaintextModulus(), bredparams)
			U[k] = float64(r * s * utils.SignFloat(float64(usum), params.PlaintextModulus()))
		}

		// ---- 6) (암호 경로) 재암호화(Ucin) 송신 ----
		Upacked := bgv.NewPlaintext(params, params.MaxLevel())
		encoder.Encode(
			utils.ModVec(
				utils.RoundVec(
					utils.ScalVecMult(1/r, utils.VecDuplicate(U, m, h)), // 기존 코드 유지
				),
				params.PlaintextModulus(),
			),
			Upacked,
		)
		Ucin, _ := encryptor.EncryptNew(Upacked)

		serializedUcin, err := Ucin.MarshalBinary()
		if err != nil {
			fmt.Println("[ENC] Ucin marshal error:", err)
			return
		}
		if _, err = encW.Write(serializedUcin); err != nil {
			fmt.Println("[ENC] Ucin write error:", err)
			return
		}
		if err := encW.Flush(); err != nil {
			fmt.Println("[ENC] Ucin flush error:", err)
			return
		}
		fmt.Println("[ENC] Ucin sent. len=", len(serializedUcin))

		// ---- 7) (암호 경로) ACK 수신 ----
		ackBuf := make([]byte, 3)
		if _, err = encR.Read(ackBuf); err != nil || string(ackBuf) != "ACK" {
			fmt.Println("[ENC] ACK read error:", err)
			return
		}

		fmt.Println("[Go] loop done. elapsed:", time.Since(startLoop))
	}
}
