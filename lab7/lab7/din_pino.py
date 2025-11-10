import pinocchio as pin
import numpy as np

if __name__ == '__main__':

  # Lectura del modelo del robot a partir de URDF (parsing)
  model = pin.buildModelFromUrdf("../urdf/ur5_robot.urdf")
  data = model.createData()

  # Evitar usar e-0x
  np.set_printoptions(suppress=True, precision=3)
  
  q = np.array([0.2, 0.1, 1.5, 0.8, 0.2, 0.3])	# Configuracion articular
  dq = np.array([0.3, 0.3, 0.4, 0.3, 0.4, 0.5])  # Velocidad articular
  ddq = np.array([0.3, 0.3, 0.5, 0.1, 0.2, 0.1]) # Aceleracion articular
  
  # Arrays PyKDL
  zeros = np.zeros(model.nq)           	# Vector de ceros
  tau   = np.zeros(model.nq)           	# Para torque
  g     = np.zeros(model.nq)   		# Para la gravedad
  c     = np.zeros(model.nq)   		# Para el vector de Coriolis+centrifuga
  M     = np.zeros(model.nq,model.nq)  	# Para la matriz de inercia
  
  # Solucionando la dinamica 
  dyn_solver = pin.rnea(model, data, q, dq, ddq)
  
  
  # Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
  # y matriz M usando Pinocchio
  
  
  
  # Parte 2: Calcular vector tau usando Pinocchio
  
  
  
