// utils.go
package utils

import (
	"bufio"
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"sort"

	"github.com/tuneinsight/lattigo/v6/core/rgsw"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
)

// 디렉토리 생성 (없으면 생성)
func EnsureDir(dir string) error {
	return os.MkdirAll(dir, 0o755)
}

// Lattigo 객체 저장 (io.WriterTo 인터페이스 구현체 공통)
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

// Lattigo 객체 로드 (io.ReaderFrom 인터페이스 구현체 공통)
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

// RGSW Ciphertext Pack 저장 (예: ctF_000.dat, ctF_001.dat ...)
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

// RGSW Ciphertext Pack 로드
func LoadRGSWPack(baseDir, name string) ([]*rgsw.Ciphertext, error) {
	out := []*rgsw.Ciphertext{}
	for i := 0; ; i++ {
		fn := filepath.Join(baseDir, fmt.Sprintf("%s_%03d.dat", name, i))
		if _, err := os.Stat(fn); err != nil {
			if os.IsNotExist(err) {
				break // 파일이 없으면 루프 종료
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

// Galois Keys 일괄 저장
func SaveGaloisKeys(baseDir string, gks []*rlwe.GaloisKey, galEls []uint64) error {
	for i, gk := range gks {
		fn := filepath.Join(baseDir, fmt.Sprintf("gk_%d.dat", galEls[i]))
		if err := WriteWT(fn, gk); err != nil {
			return fmt.Errorf("save gk(%d) failed: %v", galEls[i], err)
		}
	}
	return nil
}

// Galois Keys 일괄 로드
func LoadGaloisKeys(baseDir string) ([]*rlwe.GaloisKey, error) {
	files, err := filepath.Glob(filepath.Join(baseDir, "gk_*.dat"))
	if err != nil {
		return nil, err
	}
	if len(files) == 0 {
		// 갈루아 키가 없을 수도 있음 (경고만 하거나 에러 처리)
		return nil, nil
	}
	sort.Strings(files) // 파일명 정렬
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
