#!/usr/bin/env python3
"""
🤖 Robô Self-Balance - Monitor PID com MQTT e Matplotlib
Visualização em tempo real dos dados do controlador PID via MQTT

Dependências:
pip install paho-mqtt matplotlib numpy tkinter

Autor: GitHub Copilot
Data: 2025-11-05
"""

import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk, messagebox
import json
import threading
import time
from collections import deque
import numpy as np

class PIDMonitor:
    def __init__(self):
        # Configuração MQTT
        self.MQTT_BROKER = "localhost"  # Mude para o IP do seu broker MQTT
        self.MQTT_PORT = 1883
        self.MQTT_TOPIC_DATA = "selfbalance/data"
        self.MQTT_TOPIC_CONTROL = "selfbalance/control"
        
        # Buffers de dados (máximo 500 pontos)
        self.max_points = 500
        self.timestamps = deque(maxlen=self.max_points)
        self.angles = deque(maxlen=self.max_points)
        self.errors = deque(maxlen=self.max_points)
        self.duty_cycles = deque(maxlen=self.max_points)
        
        # Estado atual do PID
        self.current_kp = 10.0
        self.current_ki = 0.5
        self.current_kd = 1.0
        
        # Flag de conexão
        self.mqtt_connected = False
        
        # Configuração da interface
        self.setup_gui()
        self.setup_mqtt()
        self.setup_plots()
        
    def setup_gui(self):
        """Configura a interface gráfica"""
        self.root = tk.Tk()
        self.root.title("🤖 Robô Self-Balance - Monitor PID")
        self.root.geometry("1200x800")
        
        # Frame principal
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Frame de controle (esquerda)
        control_frame = ttk.LabelFrame(main_frame, text="Controle PID", padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        # Status de conexão
        self.connection_label = ttk.Label(control_frame, text="🔴 Desconectado", 
                                        foreground="red", font=("Arial", 12, "bold"))
        self.connection_label.pack(pady=(0, 20))
        
        # Controles PID
        self.create_pid_controls(control_frame)
        
        # Frame do gráfico (direita)
        plot_frame = ttk.Frame(main_frame)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Embed matplotlib no tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Botão de reconexão
        reconnect_btn = ttk.Button(control_frame, text="🔄 Reconectar MQTT", 
                                 command=self.reconnect_mqtt)
        reconnect_btn.pack(pady=10)
        
    def create_pid_controls(self, parent):
        """Cria os controles deslizantes para PID"""
        
        # Kp
        kp_frame = ttk.Frame(parent)
        kp_frame.pack(fill=tk.X, pady=5)
        ttk.Label(kp_frame, text="Kp (Proporcional):").pack()
        self.kp_var = tk.DoubleVar(value=self.current_kp)
        self.kp_scale = ttk.Scale(kp_frame, from_=0, to=100, variable=self.kp_var, 
                                orient=tk.HORIZONTAL, command=self.update_kp)
        self.kp_scale.pack(fill=tk.X)
        self.kp_label = ttk.Label(kp_frame, text=f"{self.current_kp:.1f}")
        self.kp_label.pack()
        
        # Ki
        ki_frame = ttk.Frame(parent)
        ki_frame.pack(fill=tk.X, pady=5)
        ttk.Label(ki_frame, text="Ki (Integral):").pack()
        self.ki_var = tk.DoubleVar(value=self.current_ki)
        self.ki_scale = ttk.Scale(ki_frame, from_=0, to=20, variable=self.ki_var, 
                                orient=tk.HORIZONTAL, command=self.update_ki)
        self.ki_scale.pack(fill=tk.X)
        self.ki_label = ttk.Label(ki_frame, text=f"{self.current_ki:.1f}")
        self.ki_label.pack()
        
        # Kd
        kd_frame = ttk.Frame(parent)
        kd_frame.pack(fill=tk.X, pady=5)
        ttk.Label(kd_frame, text="Kd (Derivativo):").pack()
        self.kd_var = tk.DoubleVar(value=self.current_kd)
        self.kd_scale = ttk.Scale(kd_frame, from_=0, to=50, variable=self.kd_var, 
                                orient=tk.HORIZONTAL, command=self.update_kd)
        self.kd_scale.pack(fill=tk.X)
        self.kd_label = ttk.Label(kd_frame, text=f"{self.current_kd:.1f}")
        self.kd_label.pack()
        
    def update_kp(self, value):
        """Atualiza Kp"""
        self.current_kp = float(value)
        self.kp_label.config(text=f"{self.current_kp:.1f}")
        self.send_pid_command("kp", self.current_kp)
        
    def update_ki(self, value):
        """Atualiza Ki"""
        self.current_ki = float(value)
        self.ki_label.config(text=f"{self.current_ki:.1f}")
        self.send_pid_command("ki", self.current_ki)
        
    def update_kd(self, value):
        """Atualiza Kd"""
        self.current_kd = float(value)
        self.kd_label.config(text=f"{self.current_kd:.1f}")
        self.send_pid_command("kd", self.current_kd)
        
    def send_pid_command(self, param, value):
        """Envia comando PID via MQTT"""
        if self.mqtt_connected:
            command = {param: value}
            self.mqtt_client.publish(self.MQTT_TOPIC_CONTROL, json.dumps(command))
            print(f"📤 Enviado: {param} = {value}")
        
    def setup_plots(self):
        """Configura os gráficos matplotlib"""
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.suptitle('🤖 Robô Self-Balance - Monitoramento PID', fontsize=16, fontweight='bold')
        
        # Gráfico 1: Ângulo e Erro
        self.ax1.set_title('📐 Ângulo e Erro')
        self.ax1.set_ylabel('Graus (°)')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_ylim(-45, 45)
        
        # Gráfico 2: Duty Cycle
        self.ax2.set_title('⚡ Duty Cycle (PWM)')
        self.ax2.set_xlabel('Tempo (s)')
        self.ax2.set_ylabel('PWM (-255 a 255)')
        self.ax2.grid(True, alpha=0.3)
        self.ax2.set_ylim(-255, 255)
        
        # Linhas dos gráficos
        self.line_angle, = self.ax1.plot([], [], 'b-', linewidth=2, label='Ângulo')
        self.line_error, = self.ax1.plot([], [], 'r-', linewidth=2, label='Erro')
        self.line_duty, = self.ax2.plot([], [], 'g-', linewidth=2, label='Duty Cycle')
        
        self.ax1.legend()
        self.ax2.legend()
        
        plt.tight_layout()
        
    def setup_mqtt(self):
        """Configura cliente MQTT"""
        self.mqtt_client = mqtt.Client(client_id="PID_Monitor_Python")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        try:
            self.mqtt_client.connect(self.MQTT_BROKER, self.MQTT_PORT, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            messagebox.showerror("Erro MQTT", f"Falha ao conectar: {e}")
            
    def reconnect_mqtt(self):
        """Reconecta ao MQTT"""
        try:
            self.mqtt_client.disconnect()
            time.sleep(1)
            self.mqtt_client.connect(self.MQTT_BROKER, self.MQTT_PORT, 60)
            print("🔄 Tentando reconectar ao MQTT...")
        except Exception as e:
            messagebox.showerror("Erro", f"Falha na reconexão: {e}")
            
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback de conexão MQTT"""
        if rc == 0:
            self.mqtt_connected = True
            self.connection_label.config(text="🟢 Conectado", foreground="green")
            client.subscribe(self.MQTT_TOPIC_DATA)
            print(f"✅ Conectado ao MQTT! Inscrito em: {self.MQTT_TOPIC_DATA}")
        else:
            self.mqtt_connected = False
            self.connection_label.config(text="🔴 Falha na conexão", foreground="red")
            print(f"❌ Falha na conexão MQTT: {rc}")
            
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback de desconexão MQTT"""
        self.mqtt_connected = False
        self.connection_label.config(text="🔴 Desconectado", foreground="red")
        print("❌ MQTT desconectado")
        
    def on_mqtt_message(self, client, userdata, msg):
        """Callback de mensagem MQTT"""
        try:
            data = json.loads(msg.payload.decode())
            
            # Adiciona timestamp relativo
            if len(self.timestamps) == 0:
                self.start_time = data.get('timestamp', 0)
            
            relative_time = (data.get('timestamp', 0) - self.start_time) / 1000.0
            
            # Adiciona dados aos buffers
            self.timestamps.append(relative_time)
            self.angles.append(data.get('angle', 0))
            self.errors.append(data.get('error', 0))
            self.duty_cycles.append(data.get('duty_cycle', 0))
            
            # Atualiza valores atuais do PID (se fornecidos)
            if 'kp' in data:
                self.current_kp = data['kp']
            if 'ki' in data:
                self.current_ki = data['ki']
            if 'kd' in data:
                self.current_kd = data['kd']
                
        except json.JSONDecodeError:
            print("❌ Erro ao decodificar JSON")
        except Exception as e:
            print(f"❌ Erro ao processar mensagem: {e}")
            
    def animate_plots(self, frame):
        """Atualiza os gráficos (chamado pelo FuncAnimation)"""
        if len(self.timestamps) < 2:
            return self.line_angle, self.line_error, self.line_duty
            
        # Converte para numpy arrays
        times = np.array(self.timestamps)
        angles = np.array(self.angles)
        errors = np.array(self.errors)
        duties = np.array(self.duty_cycles)
        
        # Atualiza gráfico de ângulo e erro
        self.line_angle.set_data(times, angles)
        self.line_error.set_data(times, errors)
        
        # Atualiza gráfico de duty cycle
        self.line_duty.set_data(times, duties)
        
        # Ajusta limites do eixo X
        if len(times) > 0:
            self.ax1.set_xlim(times[0], times[-1] + 1)
            self.ax2.set_xlim(times[0], times[-1] + 1)
            
        return self.line_angle, self.line_error, self.line_duty
        
    def run(self):
        """Inicia a aplicação"""
        # Inicia animação dos gráficos
        self.ani = animation.FuncAnimation(self.fig, self.animate_plots, 
                                         interval=100, blit=False, cache_frame_data=False)
        
        # Inicia interface gráfica
        try:
            self.root.mainloop()
        finally:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

def main():
    """Função principal"""
    print("🚀 Iniciando Monitor PID com MQTT...")
    
    # Verifica dependências
    try:
        import paho.mqtt.client as mqtt
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError as e:
        print(f"❌ Dependência faltando: {e}")
        print("💡 Instale com: pip install paho-mqtt matplotlib numpy")
        return
        
    # Inicia monitor
    monitor = PIDMonitor()
    monitor.run()

if __name__ == "__main__":
    main()