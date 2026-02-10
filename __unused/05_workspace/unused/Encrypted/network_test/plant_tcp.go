// go_plant_bridge_enc.go  (암호 경로 제거 버전)
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
	addrPython       = "127.0.0.1:9000" // Python ↔ Go (서버)
	alphaResetDegAbs = 30.0
)

func main() {
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
		go handlePythonConn(conn)
	}
}

func handlePythonConn(conn net.Conn) {
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
	}
}
