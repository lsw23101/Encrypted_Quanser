package main

import (
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"net"
	"time"
)

var F = [][]float64{
	{0.5826, -0.5739, 0.0536, -0.0457},
	{0.1220, -0.1321, 0.0333, -0.0252},
	{9.7428, -54.8333, 4.3777, -4.5812},
	{13.3201, -58.8872, 3.3637, -3.5394},
}

var G = [][]float64{
	{0.5524, 0.0679},
	{0.0119, 0.6530},
	{3.8233, 3.9944},
	{0.1897, 10.5578},
}

var H = []float64{-13.5083, 53.6468, -3.3636, 4.5917}

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

func matVecMult(mat [][]float64, vec []float64) []float64 {
	result := make([]float64, len(mat))
	for i, row := range mat {
		for j, v := range row {
			result[i] += v * vec[j]
		}
	}
	return result
}

func vecAdd(a, b []float64) []float64 {
	result := make([]float64, len(a))
	for i := range a {
		result[i] = a[i] + b[i]
	}
	return result
}

func dot(h, x []float64) float64 {
	var sum float64
	for i := range h {
		sum += h[i] * x[i]
	}
	return sum
}

func main() {
	n, p := 4, 2

	fmt.Println("--- Non-Encrypted Controller ---")

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

		x := make([]float64, n)
		go handleControl(conn, x, n, p)
	}
}

func handleControl(conn net.Conn, x []float64, n, p int) {
	defer conn.Close()
	iter := 0

	fmt.Println(">> Controller Loop Started")

	for {
		start := time.Now()

		// 1. u = H @ x 계산 후 전송 (Plant보다 먼저)
		u := dot(H, x)
		if err := writeFloats(conn, []float64{u}); err != nil {
			fmt.Println("Write u Error:", err)
			return
		}

		// 2. y 수신 (Plant → Controller)
		y, err := readFloats(conn, p)
		if err != nil {
			if err != io.EOF {
				fmt.Println("Read y Error:", err)
			}
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
