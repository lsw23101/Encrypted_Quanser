package com_utils

import (
	"bufio"
	"errors"
	"fmt"
	"io"
	"net"
	"os"
	"path/filepath"
	"sort"

	"github.com/tuneinsight/lattigo/v6/core/rgsw"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
)

func ReadFullData(conn net.Conn, expectedSize int) ([]byte, error) {
	totalData := make([]byte, 0, expectedSize)
	buf := make([]byte, 726) // 청크 크기 조절 확인 필요

	for len(totalData) < expectedSize {
		n, err := conn.Read(buf)
		if err != nil {
			return nil, fmt.Errorf("수신 오류: %v", err)
		}
		totalData = append(totalData, buf[:n]...)
	}
	return totalData, nil
}

func WriteToFile(data interface{}, filename string) error {
	// "enc_data" 폴더가 없으면 생성
	err := os.MkdirAll("enc_data", os.ModePerm)
	if err != nil {
		return fmt.Errorf("폴더 생성 오류: %v", err)
	}

	// 파일 경로에 "enc_data/" 폴더 추가
	filePath := fmt.Sprintf("enc_data/%s", filename)

	// 파일 생성
	file, err := os.Create(filePath)
	if err != nil {
		return fmt.Errorf("파일 생성 오류: %v", err)
	}
	defer file.Close()

	// bufio.Writer를 사용하여 버퍼링된 writer 생성
	bufferedWriter := bufio.NewWriter(file)

	// WriteTo 메소드를 사용하여 데이터를 파일에 씁니다.
	// 여기서 data가 WriteTo 메소드를 구현한 타입이어야 함
	switch v := data.(type) {
	case *rlwe.Ciphertext:
		// Ciphertext 타입일 경우
		if _, err := v.WriteTo(bufferedWriter); err != nil {
			return fmt.Errorf("암호문 쓰기 오류: %v", err)
		}
	case *rlwe.SecretKey:
		// SecretKey 타입일 경우
		if _, err := v.WriteTo(bufferedWriter); err != nil {
			return fmt.Errorf("비밀키 쓰기 오류: %v", err)
		}
	default:
		return fmt.Errorf("지원되지 않는 타입입니다: %T", v)
	}

	// 버퍼를 플러시하여 모든 데이터가 파일에 기록되도록 합니다.
	if err := bufferedWriter.Flush(); err != nil {
		return fmt.Errorf("버퍼 플러시 오류: %v", err)
	}

	return nil
}

func ReadFromFile(filename string, data interface{}) error {
	// 파일 경로에 "enc_data/" 폴더 추가
	filePath := fmt.Sprintf("enc_data/%s", filename)

	// 파일 열기
	file, err := os.Open(filePath)
	if err != nil {
		return fmt.Errorf("파일 열기 오류: %v", err)
	}
	defer file.Close()

	// bufio.Reader를 사용하여 버퍼링된 reader 생성
	bufferedReader := bufio.NewReader(file)

	// ReadFrom 메서드를 사용하여 데이터를 파일에서 읽어옵니다.
	// 여기서 data가 ReadFrom 메소드를 구현한 타입이어야 함
	switch v := data.(type) {
	case *rlwe.Ciphertext:
		// Ciphertext 타입일 경우
		if _, err := v.ReadFrom(bufferedReader); err != nil {
			return fmt.Errorf("암호문 읽기 오류: %v", err)
		}
	case *rlwe.SecretKey:
		// SecretKey 타입일 경우
		if _, err := v.ReadFrom(bufferedReader); err != nil {
			return fmt.Errorf("비밀키 읽기 오류: %v", err)
		}
	default:
		return fmt.Errorf("지원되지 않는 타입입니다: %T", v)
	}

	return nil
}

// ====== 파일 유틸 헬퍼 추가 (main 아래 어딘가) ======
// 디렉토리 보장
func EnsureDir(dir string) error {
	return os.MkdirAll(dir, 0o755)
}

// io.WriterTo 구현 타입 공통 저장
func WriteWT(path string, obj io.WriterTo) error {
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()
	w := bufio.NewWriter(f)
	if _, err := obj.WriteTo(w); err != nil {
		return err
	}
	return w.Flush()
}

// io.ReaderFrom 구현 타입 공통 로드
func ReadRT(path string, obj io.ReaderFrom) error {
	f, err := os.Open(path)
	if err != nil {
		return err
	}
	defer f.Close()
	r := bufio.NewReader(f)
	_, err = obj.ReadFrom(r)
	return err
}

// RGSW pack (예: []*rgsw.Ciphertext) 저장
func SaveRGSWPack(baseDir, name string, pack []*rgsw.Ciphertext) error {
	if pack == nil {
		return errors.New("nil pack: " + name)
	}
	for i, ct := range pack {
		if ct == nil {
			return errors.New("nil ciphertext in pack: " + name)
		}
		fn := filepath.Join(baseDir, fmt.Sprintf("%s_%03d.dat", name, i))
		if err := WriteWT(fn, ct); err != nil {
			return fmt.Errorf("save %s[%d] failed: %w", name, i, err)
		}
	}
	return nil
}

// RGSW pack 로드 (ctName_000.dat, 001.dat ... 존재하는 만큼 읽음)
func LoadRGSWPack(baseDir, name string) ([]*rgsw.Ciphertext, error) {
	out := []*rgsw.Ciphertext{}
	for i := 0; ; i++ {
		fn := filepath.Join(baseDir, fmt.Sprintf("%s_%03d.dat", name, i))
		if _, err := os.Stat(fn); err != nil {
			if os.IsNotExist(err) {
				break
			}
			return nil, err
		}
		ct := new(rgsw.Ciphertext)
		if err := ReadRT(fn, ct); err != nil {
			return nil, fmt.Errorf("load %s[%d] failed: %w", name, i, err)
		}
		out = append(out, ct)
	}
	if len(out) == 0 {
		return nil, fmt.Errorf("no files found for %s_* in %s", name, baseDir)
	}
	return out, nil
}

// 저장해 둔 Galois keys 일괄 로드 (gk_*.dat)
func LoadGaloisKeys(baseDir string) ([]*rlwe.GaloisKey, error) {
	files, err := filepath.Glob(filepath.Join(baseDir, "gk_*.dat"))
	if err != nil {
		return nil, err
	}
	if len(files) == 0 {
		return nil, fmt.Errorf("no gk_*.dat in %s", baseDir)
	}
	sort.Strings(files) // 파일명 순서대로
	out := make([]*rlwe.GaloisKey, 0, len(files))
	for _, fn := range files {
		gk := new(rlwe.GaloisKey)
		if err := ReadRT(fn, gk); err != nil {
			return nil, fmt.Errorf("load %s failed: %w", fn, err)
		}
		out = append(out, gk)
	}
	return out, nil
}

// ReadAndParseSerial reads a line from the serial reader, and returns parsed angle and distance. 작동안함
// func ReadAndParseSerial(reader *bufio.Reader) (angle float64, distance float64, err error) {
// 	line, err := reader.ReadString('\n')
// 	if err != nil {
// 		return 0, 0, err
// 	}

// 	fmt.Printf("⚠️ 원시 시리얼 입력: [%s]\n", line) // 원시 데이터 출력

// 	line = strings.TrimSpace(line)
// 	fmt.Printf("⚠️ Trimmed line: [%s]\n", line) // 트림된 데이터 출력

// 	if line == "" || !strings.Contains(line, ",") {
// 		return 0, 0, errors.New("invalid or empty serial data")
// 	}

// 	parts := strings.Split(line, ",")
// 	if len(parts) != 2 {
// 		return 0, 0, errors.New("malformed serial data")
// 	}

// 	angle, err1 := strconv.ParseFloat(parts[0], 64)
// 	distance, err2 := strconv.ParseFloat(parts[1], 64)
// 	if err1 != nil || err2 != nil {
// 		return 0, 0, errors.New("failed to parse float values")
// 	}

// 	return angle, distance, nil
// }
