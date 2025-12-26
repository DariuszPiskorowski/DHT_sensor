# DHT_sensor

Prosty projekt: odczyt temperatury i wilgotności z czujnika DHT na ESP8266
oraz publikacja danych przez MQTT.

Opis zmian i zawartość repo:
- Szkic `sensorDHT11.ino` — odczyt DHT, wysyłanie danych co 60s.
- `.github/workflows/arduino-build.yml` — GitHub Actions do kompilacji szkicu przy push/PR.
- `.gitignore` — ignorowane pliki środowiska/kompilacji.

Lokalne zmiany w szkicu:
- `sensorDHT11.ino` korzysta z hardcodowanej listy SSID/hasła, skanuje AP i łączy
    się z najsilniejszym z listy. Publikuje `WiFi/SSID` i `WiFi/RSSI` oraz dane z czujnika.
- Usunięto obsługę EEPROM i obsługę przychodzących wiadomości — urządzenie tylko publikuje.

Szybkie komendy do lokalnej kompilacji z `arduino-cli` (Windows PowerShell):

```powershell
arduino-cli core update-index
arduino-cli core install esp8266:esp8266
arduino-cli lib install "PubSubClient"
arduino-cli lib install "DHT sensor library"
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 .
```

Jeśli używasz Arduino IDE, otwórz folder projektu i skompiluj szkic `sensorDHT11.ino`.

Uwaga: workflow GitHub Actions kompiluje dla `nodemcuv2`. Jeśli masz inną płytkę,
zmień `--fqbn` w workflow lub w lokalnej komendzie.

---

Simple code included in this repository: `sensorDHT11.ino`.
