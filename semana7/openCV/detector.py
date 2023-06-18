# Importação das bibliotecas necessárias
import cv2 as cv  
import numpy as np

# Carrega o modelo YOLO pré-treinado
model = YOLO('./semana5/best.pt') 

# Inicializa a captura de vídeo da webcam
capture = cv.VideoCapture(0)  

while True:
   # Lê um quadro (frame) da captura de vídeo
   _, frame = capture.read()  
   # Realiza a detecção de objetos no quadro utilizando o modelo YOLO
   result = model.predict(frame, conf=0.6) 
   # Exibe o quadro com as detecções
   cv.imshow('frame', result[0].plot())
   # Aguarda o pressionamento da tecla 'q' para sair do loop
   if cv.waitKey(1) == ord('q'):
      break

# Libera os recursos da captura de vídeo
capture.release()

# Fecha todas as janelas abertas pelo OpenCV
cv.destroyAllWindows()

