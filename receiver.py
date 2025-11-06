#!/usr/bin/env python3
"""
🚀 Receptor UDP para Robô Balanceador ESP32
Recebe dados do robô via WiFi e exibe em tempo real
"""
import socket
import json
import time
from datetime import datetime

# Configurações (devem coincidir com o ESP32)
UDP_IP = "0.0.0.0"  # Escuta em todas as interfaces
UDP_PORT = 8888

def main():
    print("🚀 Receptor de Dados do Robô Balanceador")
    print(f"📡 Aguardando dados na porta {UDP_PORT}...")
    print("📝 Configure o IP deste PC no código ESP32\n")
    
    # Cria socket UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    print("Timestamp       | Ângulo | Erro  | Saída | Kp  | Ki  | Kd")
    print("-" * 65)
    
    try:
        while True:
            # Recebe dados
            data, addr = sock.recvfrom(1024)
            
            try:
                # Decodifica JSON
                json_data = json.loads(data.decode())
                
                # Extrai dados
                timestamp = json_data.get('timestamp', 0)
                angle = json_data.get('angle', 0)
                error = json_data.get('error', 0)
                output = json_data.get('output', 0)
                kp = json_data.get('kp', 0)
                ki = json_data.get('ki', 0)
                kd = json_data.get('kd', 0)
                
                # Formata timestamp
                time_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                
                # Imprime dados formatados
                print(f"{time_str} | {angle:6.2f} | {error:5.2f} | {output:5.0f} | {kp:3.1f} | {ki:3.1f} | {kd:3.1f}")
                
            except json.JSONDecodeError:
                print(f"❌ Erro ao decodificar JSON de {addr}")
                
    except KeyboardInterrupt:
        print("\n🛑 Parando receptor...")
    finally:
        sock.close()

if __name__ == "__main__":
    main()