#!/bin/bash

if [ "$#" -ne 3 ]; then
  echo "用法: $0 <yourfile.p12> <p12匯入密碼> <私鑰輸出密碼>"
  exit 1
fi

P12_FILE="$1"
P12_PASSWORD="$2"
KEY_PASSWORD="$3"

P12_DIR=$(dirname "$P12_FILE")
BASENAME=$(basename "$P12_FILE" .p12)

CERT_FILE="${P12_DIR}/${BASENAME}_cert.pem"
KEY_FILE="${P12_DIR}/${BASENAME}_key.pem"

openssl pkcs12 -in "$P12_FILE" -out "$CERT_FILE" -clcerts -nokeys -passin pass:"$P12_PASSWORD"
openssl pkcs12 -in "$P12_FILE" -out "$KEY_FILE" -nocerts -passin pass:"$P12_PASSWORD" -passout pass:"$KEY_PASSWORD"

echo "✅ 檔案已產生於："
echo "- 憑證: $CERT_FILE"
echo "- 私鑰（已加密）: $KEY_FILE"
