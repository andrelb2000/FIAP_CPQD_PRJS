#include "Crypto.h"
#include <Arduino.h>
#include <mbedtls/pk.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>

Crypto::Crypto() : hasKey(false) {
  hasSymKey = false;
  mbedtls_pk_init(&pk);
  mbedtls_entropy_init(&entropy);
  mbedtls_ctr_drbg_init(&ctr_drbg);
}

Crypto::~Crypto() {
  mbedtls_pk_free(&pk);
  mbedtls_ctr_drbg_free(&ctr_drbg);
  mbedtls_entropy_free(&entropy);
}

void Crypto::hexToBytes(const String &hex, std::vector<uint8_t> &out) {
  out.clear();
  int len = hex.length();
  for (int i = 0; i + 1 < len; i += 2) {
    char hi = hex.charAt(i);
    char lo = hex.charAt(i + 1);
    uint8_t vhi = (hi >= '0' && hi <= '9') ? hi - '0' : (hi >= 'a' && hi <= 'f') ? hi - 'a' + 10 : (hi >= 'A' && hi <= 'F') ? hi - 'A' + 10 : 0;
    uint8_t vlo = (lo >= '0' && lo <= '9') ? lo - '0' : (lo >= 'a' && lo <= 'f') ? lo - 'a' + 10 : (lo >= 'A' && lo <= 'F') ? lo - 'A' + 10 : 0;
    out.push_back((uint8_t)((vhi << 4) | vlo));
  }
}

bool Crypto::fetchPubKey(const String &pubkeyUrl) {
  HTTPClient http;
  http.begin(pubkeyUrl);
  Serial.print("Crypto: buscando pubkey em "); Serial.println(pubkeyUrl);
  int code = http.GET();
  if (code <= 0) {
    Serial.print("Crypto: falha ao obter pubkey, HTTP code: "); Serial.println(code);
    http.end();
    return false;
  }
  String body = http.getString();
  http.end();
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    Serial.print("Crypto: JSON pubkey parse error: "); Serial.println(err.c_str());
    return false;
  }
  if (!doc.containsKey("pubkey")) {
    Serial.println("Crypto: resposta nao contem campo 'pubkey'");
    return false;
  }
  String hexKey = doc["pubkey"].as<String>();
  version = doc.containsKey("version") ? doc["version"].as<String>() : String();

  std::vector<uint8_t> keybin;
  hexToBytes(hexKey, keybin);

  // Se o servidor retornou exatamente 16 bytes, trate como chave simétrica (toy)
  if (keybin.size() == 16) {
    hasSymKey = true;
    symKey = keybin;
    version = doc.containsKey("version") ? doc["version"].as<String>() : String();
    Serial.println("Crypto: chave simetrica 16 bytes carregada (modo toy cipher)");
    return true;
  }

  // Se a chave decodificada parecer estar em PEM (texto com -----BEGIN),
  // certifique-se de que ela esteja terminada com '\0' para que mbedtls
  // reconheça corretamente o PEM.
  bool looksLikePem = false;
  if (!keybin.empty()) {
    // busca seq "BEGIN" nos primeiros bytes
    std::string head((const char*)keybin.data(), std::min<size_t>(keybin.size(), 64));
    if (head.find("BEGIN") != std::string::npos || head.rfind("-----", 0) == 0) {
      looksLikePem = true;
    }
  }
  if (looksLikePem) {
    // adiciona terminador nulo necessário para parsing PEM
    keybin.push_back(0);
    Serial.println("Crypto: chave parece PEM; adicionando terminador nulo para parse");
  }

  int ret = mbedtls_pk_parse_public_key(&pk, keybin.data(), keybin.size());
  if (ret != 0) {
    // Exibe informações detalhadas para diagnosticar formato recebido
    Serial.print("Crypto: falha ao parsear chave pública mbedtls ret="); Serial.println(ret);
    // imprime mensagem amigável do mbedtls
    char errbuf[200];
    mbedtls_strerror(ret, errbuf, sizeof(errbuf));
    Serial.print("Crypto: mbedtls_strerror: "); Serial.println(errbuf);
    // imprime tamanho e pré-visualização hex/texto
    Serial.print("Crypto: keybin length="); Serial.println((int)keybin.size());
    Serial.print("Crypto: key preview (first 128 bytes hex): ");
    for (size_t i = 0; i < keybin.size() && i < 128; ++i) {
      char hb[3];
      const char *hex = "0123456789abcdef";
      hb[0] = hex[(keybin[i] >> 4) & 0xF]; hb[1] = hex[keybin[i] & 0xF]; hb[2] = '\0';
      Serial.print(hb);
    }
    Serial.println();
    // se os primeiros bytes são legíveis, imprime como texto para inspecao
    Serial.print("Crypto: key preview (text, printable chars): ");
    String sPreview;
    for (size_t i = 0; i < keybin.size() && i < 200; ++i) {
      char c = (char)keybin[i];
      if (c >= 32 && c <= 126) sPreview += c;
      else sPreview += '.';
    }
    Serial.println(sPreview);
    return false;
  }

  // Inicializa RNG
  const char *pers = "esp32_crypto";
  ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy, (const unsigned char*)pers, strlen(pers));
  if (ret != 0) {
    Serial.print("Crypto: falha ao inicializar RNG ret="); Serial.println(ret);
    return false;
  }
  hasKey = true;
  Serial.print("Crypto: chave publica carregada, versao="); Serial.println(version);
  return true;
}

bool Crypto::encryptString(const String &plaintext, String &outHex) {
  outHex = String();
  if (!hasKey && !hasSymKey) {
    Serial.println("Crypto: nenhuma chave carregada (publica ou simetrica)");
    return false;
  }
  // Se temos chave simetrica de 16 bytes: toy XOR cipher
  if (hasSymKey) {
    const uint8_t *k = symKey.data();
    int klen = (int)symKey.size();
    outHex.reserve(plaintext.length() * 2 + 1);
    for (size_t i = 0; i < (size_t)plaintext.length(); ++i) {
      uint8_t p = (uint8_t)plaintext[i];
      uint8_t c = p ^ k[i % klen];
      const char *hex = "0123456789abcdef";
      char b1 = hex[(c >> 4) & 0xF];
      char b2 = hex[c & 0xF];
      outHex += b1;
      outHex += b2;
    }
    return true;
  }
  size_t keylen = mbedtls_pk_get_len(&pk);
  if (keylen == 0) {
    Serial.println("Crypto: tamanho de chave invalido");
    return false;
  }
  std::vector<uint8_t> outbuf(keylen);
  size_t olen = 0;
  int ret = mbedtls_pk_encrypt(&pk,
                               (const unsigned char*)plaintext.c_str(), plaintext.length(),
                               outbuf.data(), &olen, outbuf.size(),
                               mbedtls_ctr_drbg_random, &ctr_drbg);
  if (ret != 0) {
    Serial.print("Crypto: falha encrypt ret="); Serial.println(ret);
    return false;
  }
  // converte para hex
  outHex.reserve(olen * 2 + 1);
  for (size_t i = 0; i < olen; ++i) {
    uint8_t v = outbuf[i];
    char buf[3];
    const char *hex = "0123456789abcdef";
    buf[0] = hex[(v >> 4) & 0xF];
    buf[1] = hex[v & 0xF];
    buf[2] = '\0';
    outHex += String(buf);
  }
  return true;
}
