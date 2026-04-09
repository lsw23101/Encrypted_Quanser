package main

import (
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"net"
)

// ── Dimensions ─────────────────────────────────────────────────────────
const (
	N = 4 // state dim (z)
	M = 2 // input dim (u)
	L = 2 // output dim (y), reference dim (r)
)

// ── Run parameters ─────────────────────────────────────────────────────
const (
	HOST            = "localhost"
	PORT            = "8000"
	FREQUENCY       = 50
	VMAX            = 15.0
	REF_PITCH_RAD   = 30.0 * math.Pi / 180.0
	REF_YAW_RAD     = -30.0 * math.Pi / 180.0
	REF_HALF_PERIOD = 10.0 // seconds per half-cycle
)

// ── Controller matrices (paste from MATLAB conversion.m output) ─────────
// z(k+1) = F_*z + G_*y + R_*u + P_*r
// u(k)   = H_*z + N_bar*r

var F_ = [][]float64{ // 4x4 nilpotent integer
	{0, 0, 1, 0},
	{0, 0, 0, 1},
	{0, 0, 0, 0},
	{0, 0, 0, 0},
}

var G_ = [][]float64{ // 4x2
	{2.2322, 0.3405},
	{0.2195, 3.1594},
	{-2.1659, -0.3193},
	{-0.2024, -3.0236},
}

var R_ = [][]float64{ // 4x2
	{-0.0165, -0.0221},
	{0.0208, -0.0329},
	{0.0063, 0.0083},
	{-0.0076, 0.0120},
}

var H_ = [][]float64{ // 2x4
	{-48.5698, 31.2053, 0, 0},
	{-30.3801, -22.3527, 0, 0},
}

var P_ = [][]float64{ // 4x2
	{2.3631, 0.5760},
	{0.4553, 3.4218},
	{-0.9154, -0.2199},
	{-0.1737, -1.2919},
}

var N_bar = [][]float64{ // 2x2
	{64.7566, -52.3833},
	{53.0017, 62.1050},
}

// ── TCP helpers ─────────────────────────────────────────────────────────

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

// ── Math helpers ────────────────────────────────────────────────────────

func matVec(mat [][]float64, vec []float64) []float64 {
	result := make([]float64, len(mat))
	for i, row := range mat {
		for j, v := range row {
			result[i] += v * vec[j]
		}
	}
	return result
}

func vecAdd(vecs ...[]float64) []float64 {
	result := make([]float64, len(vecs[0]))
	for _, v := range vecs {
		for i := range result {
			result[i] += v[i]
		}
	}
	return result
}

func clipVec(v []float64, limit float64) []float64 {
	out := make([]float64, len(v))
	for i, x := range v {
		if x > limit {
			out[i] = limit
		} else if x < -limit {
			out[i] = -limit
		} else {
			out[i] = x
		}
	}
	return out
}

// ── Main ────────────────────────────────────────────────────────────────

func main() {
	fmt.Println("--- Aero2 Plaintext TCP Controller ---")

	listener, err := net.Listen("tcp", HOST+":"+PORT)
	if err != nil {
		panic(err)
	}
	defer listener.Close()
	fmt.Printf("Waiting for Plant at %s:%s...\n", HOST, PORT)

	for {
		conn, err := listener.Accept()
		if err != nil {
			fmt.Println("Accept error:", err)
			continue
		}
		fmt.Println(">> Plant connected!")
		go handleControl(conn)
	}
}

func handleControl(conn net.Conn) {
	defer conn.Close()

	// 통신 흐름 (매 스텝):
	//   Controller: [1] u 계산 → [2] send u → [3] recv (u_echo, y) → [4] z 업데이트 → 반복
	//   Plant:      [1] recv u → [2] apply clip(u) → [3] read y → [4] send (u, y)    → 반복

	z := make([]float64, N) // observer state, zero init
	iter := 0

	fmt.Println(">> Controller loop started")

	for {
		// 1. 사각파 레퍼런스 (10초 주기)
		halfCycle := int(float64(iter) / (REF_HALF_PERIOD * FREQUENCY))
		refSign   := 1.0 - 2.0*float64(halfCycle%2)
		r         := []float64{refSign * REF_PITCH_RAD, refSign * REF_YAW_RAD}

		// 2. u = H_*z + N_bar*r  (클리핑 없음 — 클리핑은 Plant 하드웨어 직전에만)
		u := vecAdd(matVec(H_, z), matVec(N_bar, r))

		// 3. send u → Plant
		if err := writeFloats(conn, u); err != nil {
			fmt.Println("Send error:", err)
			return
		}

		// 4. recv [u_echo[0], u_echo[1], y[0], y[1]] from Plant (M+L floats)
		//    Plant이 실제로 받은 u(클리핑 전)와 측정값 y를 함께 전송
		data, err := readFloats(conn, M+L)
		if err != nil {
			if err != io.EOF {
				fmt.Println("Recv error:", err)
			}
			return
		}
		uEcho := data[:M]
		y     := data[M:]

		// 5. z = F_*z + G_*y + R_*u_echo + P_*r
		//    state 업데이트: 이번 스텝의 u, r과 Plant에서 받은 y 사용
		z = vecAdd(matVec(F_, z), matVec(G_, y), matVec(R_, uEcho), matVec(P_, r))

		if iter%50 == 0 {
			fmt.Printf("k=%4d | u=[%6.2f, %6.2f] V | y=[%6.2f, %6.2f] deg | r=[%5.1f, %5.1f] deg\n",
				iter, u[0], u[1],
				y[0]*180/math.Pi, y[1]*180/math.Pi,
				r[0]*180/math.Pi, r[1]*180/math.Pi)
		}
		iter++
	}
}
