#!/usr/bin/env bash
# ONNX Runtime (CPU) 를 이 디렉터리에 내려받는다.
#
# 바이너리를 레포에 커밋하지 않고 이 스크립트로 받는 이유:
#   - 레포가 가볍게 유지된다
#   - 버전이 스크립트에 명시로 남는다 (바이너리에는 남지 않는다)
#   - 체크섬을 검증하므로 받은 것이 의도한 것임을 보장한다
#
# 로봇 머신에서 최초 1회만 네트워크가 필요하다. Makefile 이 $ORIGIN 상대 rpath 로
# 링크하므로 시스템 설치는 필요 없다.
#
# 사용법:  ./fetch.sh
set -euo pipefail

VERSION="1.23.2"
SHA256="1fa4dcaef22f6f7d5cd81b28c2800414350c10116f5fdd46a2160082551c5f9b"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAME="onnxruntime-linux-x64-${VERSION}"
URL="https://github.com/microsoft/onnxruntime/releases/download/v${VERSION}/${NAME}.tgz"
TGZ="${HERE}/${NAME}.tgz"

if [ -f "${HERE}/lib/libonnxruntime.so" ] && [ -f "${HERE}/include/onnxruntime_cxx_api.h" ]; then
    echo "[fetch] 이미 설치돼 있습니다 (v${VERSION}). 다시 받으려면 include/ lib/ 를 지우세요."
    exit 0
fi

echo "[fetch] 내려받는 중: ${NAME}"
curl -fsSL -o "${TGZ}" "${URL}"

echo "[fetch] 체크섬 검증"
echo "${SHA256}  ${TGZ}" | sha256sum -c -

echo "[fetch] 압축 해제"
tar xzf "${TGZ}" -C "${HERE}" --strip-components=1 "${NAME}/include" "${NAME}/lib"
rm -f "${TGZ}"

echo "[fetch] 완료:"
echo "        $(ls -1 "${HERE}/lib/"libonnxruntime.so* | head -1)"
echo "        $(du -sh "${HERE}/lib" | cut -f1) (lib)"
