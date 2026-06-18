package main

/*
#include <stdlib.h>
*/
import "C"

// 암호 라이브러리를 파이썬에서 사용
// go build -o client_crypto.dll -buildmode=c-shared lib_crypto.go  (Windows)
// go build -o client_crypto.so  -buildmode=c-shared lib_crypto.go  (Linux/macOS)
// 파라미터는 offline.go 가 생성하는 enc_data/params.json 에서 읽습니다.

import (
	"encoding/json"
	"math"
	"os"
	"path/filepath"
	"unsafe"

	fileutils "github.com/CDSL-EncryptedControl/CDSL/EncComm/utils"
	utils "github.com/CDSL-EncryptedControl/CDSL/utils"
	RLWE "github.com/CDSL-EncryptedControl/CDSL/utils/core/RLWE"
	"github.com/tuneinsight/lattigo/v6/core/rlwe"
	"github.com/tuneinsight/lattigo/v6/ring"
)

var (
	params    rlwe.Parameters
	encryptor *rlwe.Encryptor
	decryptor *rlwe.Decryptor
	ringQ     *ring.Ring

	s, L, r float64
	n, m, p int
	tau     int
)

type cryptoConfig struct {
	LogN    int     `json:"logN"`
	LogQ    []int   `json:"logQ"`
	LogP    []int   `json:"logP"`
	NTTFlag bool    `json:"nttFlag"`
	S       float64 `json:"s"`
	L       float64 `json:"L"`
	R       float64 `json:"r"`
	N       int     `json:"n"`
	M       int     `json:"m"`
	P       int     `json:"p"`
}

// InitCrypto 는 params.json 파일 경로를 받아 암호문 파라미터를 초기화합니다.
// Python 에서 호출: lib.InitCrypto(params_json_path.encode())
//
//export InitCrypto
func InitCrypto(configPathC *C.char) {
	configPath := C.GoString(configPathC)

	data, err := os.ReadFile(configPath)
	if err != nil {
		panic("params.json 로드 실패 (offline.go 를 먼저 실행하세요): " + err.Error())
	}
	var cfg cryptoConfig
	if err := json.Unmarshal(data, &cfg); err != nil {
		panic("params.json 파싱 실패: " + err.Error())
	}

	n = cfg.N
	m = cfg.M
	p = cfg.P
	s = cfg.S
	L = cfg.L
	r = cfg.R

	params, err = rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN:    cfg.LogN,
		LogQ:    cfg.LogQ,
		LogP:    cfg.LogP,
		NTTFlag: cfg.NTTFlag,
	})
	if err != nil {
		panic(err)
	}
	ringQ = params.RingQ()

	maxDim := math.Max(math.Max(float64(n), float64(m)), float64(p))
	tau = int(math.Pow(2, math.Ceil(math.Log2(maxDim))))

	// 비밀키 로드 (params.json 과 같은 enc_data 디렉토리)
	loadDir := filepath.Dir(configPath)
	sk := new(rlwe.SecretKey)
	if err := fileutils.ReadRT(filepath.Join(loadDir, "sk.dat"), sk); err != nil {
		panic("Key Load Fail: " + err.Error())
	}

	encryptor = rlwe.NewEncryptor(params, sk)
	decryptor = rlwe.NewDecryptor(params, sk)
}

//export EncryptVector
func EncryptVector(valuesPtr *C.double, length C.int, sizePtr *C.int) *C.char {
	cSlice := (*[1 << 30]float64)(unsafe.Pointer(valuesPtr))[:length:length]

	vec := make([]float64, int(length))
	copy(vec, cSlice)

	vecBar := utils.RoundVec(utils.ScalVecMult(1/r, vec))
	ctPack := RLWE.EncPack(vecBar, tau, 1/L, *encryptor, ringQ, params)

	data, err := ctPack.MarshalBinary()
	if err != nil {
		return nil
	}

	*sizePtr = C.int(len(data))
	return (*C.char)(C.CBytes(data))
}

//export DecryptVector
func DecryptVector(dataPtr unsafe.Pointer, dataLen C.int, outLen C.int) *C.double {
	data := C.GoBytes(dataPtr, dataLen)

	ctPack := new(rlwe.Ciphertext)
	if err := ctPack.UnmarshalBinary(data); err != nil {
		return nil
	}

	targetDim := int(outLen)
	decVec := RLWE.DecUnpack(ctPack, targetDim, tau, *decryptor, r*s*s*L, ringQ, params)

	res := (*C.double)(C.malloc(C.size_t(8 * targetDim)))

	start := unsafe.Pointer(res)
	size := unsafe.Sizeof(float64(0))

	for i := 0; i < targetDim; i++ {
		ptr := unsafe.Pointer(uintptr(start) + uintptr(i)*size)
		*(*float64)(ptr) = 0.0
		if i < len(decVec) {
			*(*float64)(ptr) = decVec[i]
		}
	}

	return res
}

//export FreePtr
func FreePtr(ptr unsafe.Pointer) {
	C.free(ptr)
}

func main() {}
