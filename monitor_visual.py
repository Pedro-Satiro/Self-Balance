#!/usr/bin/env python3
"""
📊 Monitor Visual para Robô Balanceador ESP32
Recebe dados via UDP e exibe gráficos em tempo real
"""
import socket
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import time

# Configurações
UDP_IP = "0.0.0.0"
UDP_PORT = 8888
MAX_POINTS = 200  # Últimos 200 pontos (20 segundos a 10Hz)

class RobotMonitor:
    def __init__(self):
        # Dados armazenados
        self.timestamps = deque(maxlen=MAX_POINTS)
        self.angles = deque(maxlen=MAX_POINTS)
        self.errors = deque(maxlen=MAX_POINTS)
        self.outputs = deque(maxlen=MAX_POINTS)
        
        # Status
        self.last_data_time = time.time()
        self.data_lock = threading.Lock()
        
        # Socket UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0)  # Timeout de 1 segundo
        
        # Setup da figura
        self.setup_plot()
        
    def setup_plot(self):
        """Configura os gráficos"""
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(12, 8))
        self.fig.suptitle('🚀 Monitor do Robô Balanceador ESP32', fontsize=16)
        
        # Gráfico 1: Ângulo
        self.line1, = self.ax1.plot([], [], 'b-', linewidth=2, label='Ângulo (°)')
        self.ax1.axhline(y=0, color='r', linestyle='--', alpha=0.5, label='Target (0°)')
        self.ax1.set_ylabel('Ângulo (°)')
        self.ax1.set_ylim(-45, 45)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.legend()
        
        # Gráfico 2: Erro
        self.line2, = self.ax2.plot([], [], 'r-', linewidth=2, label='Erro PID')
        self.ax2.axhline(y=0, color='g', linestyle='--', alpha=0.5)
        self.ax2.set_ylabel('Erro (°)')
        self.ax2.set_ylim(-30, 30)
        self.ax2.grid(True, alpha=0.3)
        self.ax2.legend()
        
        # Gráfico 3: Saída do Motor
        self.line3, = self.ax3.plot([], [], 'g-', linewidth=2, label='Saída Motor')
        self.ax3.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        self.ax3.set_ylabel('PWM')
        self.ax3.set_xlabel('Tempo (s)')
        self.ax3.set_ylim(-255, 255)
        self.ax3.grid(True, alpha=0.3)
        self.ax3.legend()
        
        plt.tight_layout()
        
    def udp_listener(self):
        """Thread para receber dados UDP"""
        print("📡 Iniciando receptor UDP...")
        
        try:
            self.sock.bind((UDP_IP, UDP_PORT))
            print(f"✅ Escutando na porta {UDP_PORT}")
            
            while True:
                try:
                    data, addr = self.sock.recvfrom(1024)
                    json_data = json.loads(data.decode())
                    
                    with self.data_lock:
                        current_time = time.time()
                        
                        # Adiciona dados
                        self.timestamps.append(current_time)
                        self.angles.append(json_data.get('angle', 0))
                        self.errors.append(json_data.get('error', 0))
                        self.outputs.append(json_data.get('output', 0))
                        
                        self.last_data_time = current_time
                        
                except socket.timeout:
                    # Timeout normal, continua
                    pass
                except json.JSONDecodeError:
                    print("❌ Erro ao decodificar JSON")
                    
        except Exception as e:
            print(f"❌ Erro no receptor UDP: {e}")
            
    def animate(self, frame):
        """Atualiza os gráficos"""
        with self.data_lock:
            if not self.timestamps:
                return self.line1, self.line2, self.line3
                
            # Converte timestamps para tempo relativo
            if self.timestamps:
                start_time = self.timestamps[0]
                times = [(t - start_time) for t in self.timestamps]
                
                # Atualiza gráficos
                self.line1.set_data(times, self.angles)
                self.line2.set_data(times, self.errors)
                self.line3.set_data(times, self.outputs)
                
                # Ajusta eixo X
                if times:
                    x_min, x_max = max(0, times[-1] - 20), times[-1] + 1
                    for ax in [self.ax1, self.ax2, self.ax3]:
                        ax.set_xlim(x_min, x_max)
                
                # Verifica se ainda está recebendo dados
                time_since_last = time.time() - self.last_data_time
                status = "🟢 Conectado" if time_since_last < 2 else "🔴 Desconectado"
                self.fig.suptitle(f'🚀 Monitor do Robô Balanceador ESP32 - {status}', fontsize=16)
        
        return self.line1, self.line2, self.line3
    
    def start(self):
        """Inicia o monitor"""
        # Inicia thread UDP
        udp_thread = threading.Thread(target=self.udp_listener, daemon=True)
        udp_thread.start()
        
        # Inicia animação
        ani = animation.FuncAnimation(
            self.fig, self.animate, interval=100, blit=False, cache_frame_data=False
        )
        
        print("📊 Exibindo gráficos em tempo real...")
        print("🛑 Feche a janela para sair")
        
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.sock.close()

if __name__ == "__main__":
    print("📊 Monitor Visual do Robô Balanceador")
    print("📝 Configure o IP deste PC no código ESP32")
    print("🔧 Instale dependências: pip install matplotlib")
    print()
    
    monitor = RobotMonitor()
    monitor.start()