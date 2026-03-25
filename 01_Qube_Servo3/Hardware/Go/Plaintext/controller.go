package main

import (
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"net"
	"time"
)

// F: 4x4 상태 전이 행렬
var F = [][]float64{
	{0.6951, -0.8117, 0.0756, -0.0696},
	{0.2336, -0.3680, 0.0552, -0.0489},
	{21.0411, -78.7214, 6.5861, -6.9828},
	{24.5716, -82.6764, 5.5629, -5.9310},
}

// G: 4x2 이득 행렬
var G = [][]float64{
	{0.5524, 0.0679},
	{0.0119, 0.6530},
	{3.8233, 3.9944},
	{0.1897, 10.5578},
}

// H: 1x4 출력 행렬 (u = H @ x)
var H = []float64{24.7585, -77.4332, 5.5625, -6.9830}

// float64 배열 전송: [4-byte length | 8*n bytes data]
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

// float64 배열 수신: [4-byte length | 8*count bytes data]
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

// 행렬-벡터 곱
func matVecMult(mat [][]float64, vec []float64) []float64 {
	result := make([]float64, len(mat))
	for i, row := range mat {
		for j, v := range row {
			result[i] += v * vec[j]
		}
	}
	return result
}

// 벡터 덧셈
func vecAdd(a, b []float64) []float64 {
	result := make([]float64, len(a))
	for i := range a {
		result[i] = a[i] + b[i]
	}
	return result
}

// 내적 (H @ x → scalar)
func dot(h, x []float64) float64 {
	var sum float64
	for i := range h {
		sum += h[i] * x[i]
	}
	return sum
}

func main() {
	n, p := 4, 2

	fmt.Println("--- Plaintext Controller ---")

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

		x := make([]float64, n) // 초기 상태 [0,0,0,0]
		go handleControl(conn, x, n, p)
	}
}

func handleControl(conn net.Conn, x []float64, n, p int) {
	defer conn.Close()
	iter := 0

	fmt.Println(">> Controller Loop Started")

	for {
		start := time.Now()

		// 1. y 수신 (Plant → Controller)
		y, err := readFloats(conn, p)
		if err != nil {
			if err != io.EOF {
				fmt.Println("Read y Error:", err)
			}
			return
		}

		// 2. u = H @ x 계산 후 전송
		u := dot(H, x)
		if err := writeFloats(conn, []float64{u}); err != nil {
			fmt.Println("Write u Error:", err)
			return
		}

		// 3. 상태 업데이트: x = F @ x + G @ y
		Fx := matVecMult(F, x)
		Gy := matVecMult(G, y)
		x = vecAdd(Fx, Gy)

		elapsed := time.Since(start).Milliseconds()
		if iter%100 == 0 {
			fmt.Printf("Iter %d: Cycle Time %d ms\n", iter, elapsed)
		}
		iter++
	}
}
