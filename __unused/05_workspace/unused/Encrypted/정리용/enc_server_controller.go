// controller_server.go
package main

import (
	"bufio"
	"fmt"
	"math"
	"net"
	"strconv"
	"strings"
	"time"
)

// --- 컨트롤러 행렬 (Python과 동일) ---
var F = [4][4]float64{
	{0.6951, -0.8117, 0.0756, -0.0696},
	{0.2336, -0.3680, 0.0552, -0.0489},
	{21.0411, -78.7214, 6.5861, -6.9828},
	{24.5716, -82.6764, 5.5629, -5.9310},
}
var G = [4][2]float64{
	{0.5524, 0.0679},
	{0.0119, 0.6530},
	{3.8233, 3.9944},
	{0.1897, 10.5578},
}
var H = [4]float64{24.7585, -77.4332, 5.5625, -6.9830}

const (
	addr             = "127.0.0.1:9000"
	alphaResetDegAbs = 5.0 // |alpha| > 5 deg → xhat = 0
)

func main() {
	ln, err := net.Listen("tcp", addr)
	if err != nil {
		panic(err)
	}
	fmt.Println("[Go] Controller server listening on", addr)
	for {
		conn, err := ln.Accept()
		if err != nil {
			fmt.Println("[Go] accept error:", err)
			continue
		}
		// Nagle 비활성화
		if tcp, ok := conn.(*net.TCPConn); ok {
			tcp.SetNoDelay(true)
			tcp.SetKeepAlive(true)
		}
		go handleConn(conn)
	}
}

func handleConn(conn net.Conn) {
	defer conn.Close()
	fmt.Println("[Go] New client from", conn.RemoteAddr())

	r := bufio.NewReaderSize(conn, 64*1024)
	w := bufio.NewWriterSize(conn, 64*1024)

	// 연결별 상태 xhat
	var xhat [4]float64
	var nLines int

	// 간단한 속도 모니터링
	last := time.Time{}

	for {
		// 한 줄 읽기: "theta,alpha\n"
		line, err := r.ReadString('\n')
		if err != nil {
			fmt.Println("[Go] read error:", err)
			return
		}
		s := strings.TrimSpace(line)
		if s == "" {
			continue
		}
		parts := strings.SplitN(s, ",", 3)
		if len(parts) < 2 {
			fmt.Println("[Go] malformed line:", s)
			continue
		}
		theta, err0 := strconv.ParseFloat(strings.TrimSpace(parts[0]), 64)
		alpha, err1 := strconv.ParseFloat(strings.TrimSpace(parts[1]), 64)
		if err0 != nil || err1 != nil {
			fmt.Println("[Go] parse error:", err0, err1, "line:", s)
			continue
		}

		// 주기 모니터 (옵션)
		now := time.Now()
		if !last.IsZero() {
			dt := now.Sub(last).Seconds() * 1000
			if nLines%50 == 0 {
				fmt.Printf("[Go] loop dt ≈ %.3f ms\n", dt)
			}
		}
		last = now
		nLines++

		// ===== 제어기 =====
		// 0) 각도 조건에 따른 xhat 리셋 (Python과 동일)
		alphaDeg := math.Abs(alpha * 180.0 / math.Pi)
		if alphaDeg > alphaResetDegAbs {
			for i := range xhat {
				xhat[i] = 0.0
			}
		}

		// 1) u = - H xhat   (Python: u = -1 * H @ xhat)
		u := 0.0
		for i := 0; i < 4; i++ {
			u += H[i] * xhat[i]
		}
		u = -u

		// 2) xhat ← F xhat + G y
		y0, y1 := theta, alpha
		var xnext [4]float64
		for i := 0; i < 4; i++ {
			sum := 0.0
			for j := 0; j < 4; j++ {
				sum += F[i][j] * xhat[j]
			}
			sum += G[i][0]*y0 + G[i][1]*y1
			xnext[i] = sum
		}
		xhat = xnext

		// 3) 응답 송신
		//    (필요시 포맷/정밀도 변경 가능)
		if _, err := fmt.Fprintf(w, "%.6f\n", u); err != nil {
			fmt.Println("[Go] write error:", err)
			return
		}
		if err := w.Flush(); err != nil {
			fmt.Println("[Go] flush error:", err)
			return
		}
	}
}
