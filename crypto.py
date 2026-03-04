import logging
import os
import ucryptolib 
import ujson


logger = logging.getLogger(__name__)

# ==============================================================================
# Класс для симметричного шифрования (AES-128 CBC)
# ==============================================================================
class AESCryptoManager:
    def __init__(self, key_path="secret.key"):
        self.key = None
        self.block_size = 16
        self._load_or_generate_key(key_path)

    def _load_or_generate_key(self, key_path):
        try:
            with open(key_path, 'rb') as f:
                self.key = f.read(16)
            logger.info("✅ Key loaded from %s", key_path)
        except OSError:
            logger.warning("No key found. Generating new 16-byte key...")
            self.key = os.urandom(16)
            with open(key_path, 'wb') as f:
                f.write(self.key)
            logger.info("✅ New key generated and saved to %s", key_path)

    def _pad(self, data: bytes) -> bytes:
        pad_len = self.block_size - (len(data) % self.block_size)
        return data + bytes([pad_len] * pad_len)

    def _unpad(self, data: bytes) -> bytes:
        return data[:-data[-1]]

    def encrypt_json(self, data_dict) -> bytes:
        if not self.key: return None
        raw_bytes = ujson.dumps(data_dict).encode("utf-8")
        iv = os.urandom(self.block_size)
        cipher = ucryptolib.aes(self.key, 2, iv)
        return iv + cipher.encrypt(self._pad(raw_bytes))

    def decrypt_json(self, payload: bytes):
        if not self.key or len(payload) <= self.block_size: return None
        iv, enc = payload[:self.block_size], payload[self.block_size:]
        try:
            cipher = ucryptolib.aes(self.key, 2, iv)
            decrypted_raw = self._unpad(cipher.decrypt(enc))
            return ujson.loads(decrypted_raw.decode("utf-8"))
        except Exception as e:
            logger.error("❌ Decryption failed: %s", str(e))
            return None