package main

/*
#include <stdlib.h>
*/
import "C"

// 암호 라이브러리를 파이썬엣서 사용
// go build -o client_crypto.so -buildmode=c-shared lib_crypto.go

import (
	"math"
	"path/filepath"
	"unsafe"

	fileutils "github.com/CDSL-EncryptedControl/CDSL/python_test/utils"
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

	// --- [변경] 전역 변수로 선언만 하고, 값은 Init에서 채움 ---
	s, L, r float64
	n, m, p int
	tau     int
)

// [변경] 파이썬에서 시스템 차원과 파라미터를 받아옴
//
//export InitCrypto
func InitCrypto(in_n, in_m, in_p C.int, in_s, in_L, in_r C.double) {
	// 1. 전역 변수 설정
	n = int(in_n)
	m = int(in_m)
	p = int(in_p)
	s = float64(in_s)
	L = float64(in_L)
	r = float64(in_r)

	// 2. 파라미터 로드 (Lattigo 파라미터는 고정이라고 가정)
	var err error
	params, err = rlwe.NewParametersFromLiteral(rlwe.ParametersLiteral{
		LogN: 12, LogQ: []int{60}, LogP: []int{60}, NTTFlag: true,
	})
	if err != nil {
		panic(err)
	}
	ringQ = params.RingQ()

	// 3. Tau 동적 계산 (입력받은 차원 중 가장 큰 값 기준)
	maxDim := math.Max(math.Max(float64(n), float64(m)), float64(p))
	tau = int(math.Pow(2, math.Ceil(math.Log2(maxDim))))

	// 4. 키 로드
	loadDir := "../controller/enc_data"
	sk := new(rlwe.SecretKey)
	if err := fileutils.ReadRT(filepath.Join(loadDir, "sk.dat"), sk); err != nil {
		panic("Key Load Fail: " + err.Error())
	}

	// 5. 암호화기 생성
	encryptor = rlwe.NewEncryptor(params, sk)
	decryptor = rlwe.NewDecryptor(params, sk)
}

// [변경] 실수 배열 포인터와 길이를 받음 (Generic Encrypt)
//
//export EncryptVector
func EncryptVector(valuesPtr *C.double, length C.int, sizePtr *C.int) *C.char {
	// 1. C 배열 -> Go 슬라이스로 변환 (복사 없이 참조)
	// (매우 중요: 1<<30은 임의의 큰 수, 실제로는 length만큼만 슬라이싱함)
	cSlice := (*[1 << 30]float64)(unsafe.Pointer(valuesPtr))[:length:length]

	// Go 슬라이스로 복사 (안전하게)
	vec := make([]float64, int(length))
	copy(vec, cSlice)

	// 2. 암호화
	vecBar := utils.RoundVec(utils.ScalVecMult(1/r, vec))
	ctPack := RLWE.EncPack(vecBar, tau, 1/L, *encryptor, ringQ, params)

	// 3. 직렬화
	data, err := ctPack.MarshalBinary()
	if err != nil {
		return nil
	}

	*sizePtr = C.int(len(data))
	return (*C.char)(C.CBytes(data))
}

// [변경] 복호화 후 배열 반환 (길이는 전역변수 m 사용 가능하지만, 인자로 받아도 됨)
// 여기서는 편의상 m(입력차원) 만큼 복호화한다고 가정하거나,
// DecryptU 처럼 특정 용도라면 m을 사용. 범용성을 위해 요청한 크기(outLen)만큼 반환.
//
//export DecryptVector
func DecryptVector(dataPtr unsafe.Pointer, dataLen C.int, outLen C.int) *C.double {
	data := C.GoBytes(dataPtr, dataLen)

	ctPack := new(rlwe.Ciphertext)
	if err := ctPack.UnmarshalBinary(data); err != nil {
		return nil
	}

	// 복호화 (요청한 outLen 만큼)
	targetDim := int(outLen)
	decVec := RLWE.DecUnpack(ctPack, targetDim, tau, *decryptor, r*s*s*L, ringQ, params)

	// C 메모리 할당
	res := (*C.double)(C.malloc(C.size_t(8 * targetDim))) // 8 bytes * N

	// Go 슬라이스 -> C 배열 복사
	// (unsafe pointer 연산으로 순회하며 대입)
	start := unsafe.Pointer(res)
	size := unsafe.Sizeof(float64(0))

	for i := 0; i < targetDim; i++ {
		// res[i] 주소 계산
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
