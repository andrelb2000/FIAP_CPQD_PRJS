// Crypto.h
#ifndef CRYPTO_H
#define CRYPTO_H

#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "mbedtls/pk.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include <vector>

class Crypto {
public:
  Crypto();
  ~Crypto();

  // Busca a chave pública no endpoint informado (ex: "http://host:9000/pubkey").
  // Preenche `version` e prepara contexto para criptografia.
  bool fetchPubKey(const String &pubkeyUrl);

  // Criptografa a string plaintext usando a chave pública carregada.
  // Retorna `true` e preenche `outHex` com bytes cifrados em hexadecimal.
  bool encryptString(const String &plaintext, String &outHex);

  String version;

private:
  mbedtls_pk_context pk;
  mbedtls_entropy_context entropy;
  mbedtls_ctr_drbg_context ctr_drbg;
  bool hasKey;
  bool hasSymKey;
  std::vector<uint8_t> symKey; // used when server returns 16-byte symmetric key

  void hexToBytes(const String &hex, std::vector<uint8_t> &out);
};

#endif // CRYPTO_H
