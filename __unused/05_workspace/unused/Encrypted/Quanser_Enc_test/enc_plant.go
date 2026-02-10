// go_plant_bridge_enc.go
package main

import (
	"Quanser_Enc_test/com_utils"
	"bufio"
	"fmt"
	"log"
	"math"
	"net"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	utils "github.com/CDSL-EncryptedControl/CDSL/utils"
	RLWE "github.com/CDSL-EncryptedControl/CDSL/utils/core/RLWE"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
	"github.com/tuneinsight/lattigo/v6/ring"
)

// ====== (로컬 제어) Python과 동일한 상태공간/게인 ======
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

// ====== 주소 ======
const (
	addrPython        = "127.0.0.1:9000" // Python ↔ Go (서버)
	addrEncController = "127.0.0.1:9100" // Go ↔ 암호 컨트롤러 (클라)
	alphaResetDegAbs  = 30.0
)

// ====== (암호 경로) 파라미터/스케일 ======
const (
	// RLWE params
	logN = 12
	logQ = 56
	logP = 51

	// dims
	n = 4
	m = 1
	p = 2

	// quantization scales
	s      = 1.0 / 10.0
	L      = 1.0 / 10000.0
	r_quan = 1.0 / 1000.0
)

func main() {
	// ================= RLWE 세팅 & 키 로딩 =================
	params, _ := rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN:    logN,
		LogQ:    []int{logQ},
		LogP:    []int{logP},
		NTTFlag: true,
	})
	ringQ := params.RingQ() // *ring.Ring

	base := filepath.Join("Offline_task", "enc_data", "rgsw_for_N12")
	sk := new(rlwe.SecretKey)
	if err := com_utils.ReadRT(filepath.Join(base, "sk.dat"), sk); err != nil {
		log.Fatalf("load sk: %v", err)
	}
	encryptor := rlwe.NewEncryptor(params, sk)   // *rlwe.Encryptor
	decryptor := rlwe.NewDecryptor(params, sk)   // *rlwe.Decryptor

	// tau 계산
	maxDim := math.Max(math.Max(float64(n), float64(m)), float64(p))
	tau := int(math.Pow(2, math.Ceil(math.Log2(maxDim))))

	// ================ 암호 컨트롤러에 TCP 접속 (클라이언트) ================
	encConn, err := net.Dial("tcp", addrEncController)
	if err != nil {
		log.Fatalf("[Go] failed to connect to encrypted controller (%s): %v", addrEncController, err)
	}
	defer encConn.Close()
	fmt.Println("[Go] Connected to encrypted controller at", addrEncController)

	// Nagle off / 버퍼 크게
	if tcp, ok := encConn.(*net.TCPConn); ok {
		tcp.SetNoDelay(true)
		tcp.SetKeepAlive(true)
		tcp.SetReadBuffer(1 << 20)
		tcp.SetWriteBuffer(1 << 20)
	}
	encR := bufio.NewReaderSize(encConn, 256<<10)
	encW := bufio.NewWriterSize(encConn, 256<<10)

	// ================= Python 수신용 TCP 서버 오픈 =================
	ln, err := net.Listen("tcp", addrPython)
	if err != nil {
		log.Fatalf("[Go] listen error: %v", err)
	}
	defer ln.Close()
	fmt.Println("[Go] Controller server listening on", addrPython)

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
		go handlePythonConn(conn, encryptor, decryptor, ringQ, params, tau, encR, encW)
	}
}

func handlePythonConn(
	conn net.Conn,
	encryptor *rlwe.Encryptor, // 포인터 타입
	decryptor *rlwe.Decryptor, // 포인터 타입
	ringQ *ring.Ring,          // rlwe.RingQ가 아니라 *ring.Ring
	params rlwe.Parameters,
	tau int,
	encR *bufio.Reader,
	encW *bufio.Writer,
) {
	defer conn.Close()
	fmt.Println("[Go] New Python client from", conn.RemoteAddr())
	pyR := bufio.NewReaderSize(conn, 64*1024)
	pyW := bufio.NewWriterSize(conn, 64*1024)

	var xhat [4]float64
	var nLines int
	last := time.Time{}

	for {
		// ---------- 1) Python → y=[theta,alpha] ----------
		line, err := pyR.ReadString('\n')
		if err != nil {
			fmt.Println("[Go] read error:", err)
			return
		}
		lineTrim := strings.TrimSpace(line)
		if lineTrim == "" {
			continue
		}
		parts := strings.SplitN(lineTrim, ",", 3)
		if len(parts) < 2 {
			fmt.Println("[Go] malformed line:", lineTrim)
			continue
		}
		theta, err0 := strconv.ParseFloat(strings.TrimSpace(parts[0]), 64)
		alpha, err1 := strconv.ParseFloat(strings.TrimSpace(parts[1]), 64)
		if err0 != nil || err1 != nil {
			fmt.Println("[Go] parse error:", err0, err1, "line:", lineTrim)
			continue
		}

		// 주기 모니터
		now := time.Now()
		if !last.IsZero() {
			dt := now.Sub(last).Seconds() * 1000
			if nLines%50 == 0 {
				fmt.Printf("[Go] loop dt ≈ %.3f ms\n", dt)
			}
		}
		last = now
		nLines++

		// ---------- 2) (로컬) 제어 u = -H xhat ----------
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

		// xhat ← F xhat + G y
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

		// ---------- 3) (로컬) u를 Python에 반환 ----------
		if _, err := fmt.Fprintf(pyW, "%.6f\n", u); err != nil {
			fmt.Println("[Go] write error:", err)
			return
		}
		if err := pyW.Flush(); err != nil {
			fmt.Println("[Go] flush error:", err)
			return
		}

		// ======================================================
		// ============ 4) (암호 경로) y를 암호화해 전송 =========
		// ======================================================
		y := []float64{theta, alpha}
		yBar := utils.RoundVec(utils.ScalVecMult(1.0/r_quan, y))
		yCtPack := RLWE.EncPack(yBar, tau, 1.0/L, *encryptor, ringQ, params)

		// 암호문 송신
		if _, err := yCtPack.WriteTo(encW); err != nil {
			fmt.Println("[Go][enc] write yCtPack err:", err)
			return
		}
		if err := encW.Flush(); err != nil {
			fmt.Println("[Go][enc] flush err:", err)
			return
		}

		// ---------- 5) (암호 경로) uCtPack 수신 ----------
		uCtPack := new(rlwe.Ciphertext)
		if _, err := uCtPack.ReadFrom(encR); err != nil {
			fmt.Println("[Go][enc] read uCtPack err:", err)
			return
		}

		// ---------- 6) (암호 경로) 복호화 & 출력 ----------
		uVec := RLWE.DecUnpack(uCtPack, m, tau, *decryptor, r_quan*s*L, ringQ, params)
		uEnc := 0.0
		if len(uVec) > 0 {
			uEnc = uVec[0]
		}
		fmt.Printf("[Go][enc-path] decrypted result (y0+y1) = %.6f\n", uEnc)
	}
}
