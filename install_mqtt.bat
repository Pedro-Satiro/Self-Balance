@echo off
echo 🚀 Instalando MQTT Broker (Mosquitto) e dependências Python...
echo.

REM Verifica se o Chocolatey está instalado
where choco >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo ❌ Chocolatey não encontrado. Instalando...
    echo 💡 Será necessário executar como Administrador
    pause
    
    @"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -InputFormat None -ExecutionPolicy Bypass -Command "iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))" && SET "PATH=%PATH%;%ALLUSERSPROFILE%\chocolatey\bin"
)

echo ✅ Instalando Mosquitto MQTT Broker...
choco install mosquitto -y

echo ✅ Instalando dependências Python...
pip install -r requirements.txt

echo.
echo 🎉 Instalação concluída!
echo.
echo 📋 Próximos passos:
echo 1. Inicie o broker MQTT: mosquitto -v
echo 2. Compile e faça upload do código ESP32
echo 3. Execute o monitor: python pid_monitor.py
echo.
pause