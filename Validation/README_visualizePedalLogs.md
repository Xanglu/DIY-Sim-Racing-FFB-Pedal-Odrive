# Record pedal state logs
1) Open the Simhub plugin and connect to the pedal.
2) Select "Debug Flag 0" = 64 <br> <img width="200" alt="Bildschirmfoto 2024-11-01 um 20 37 07" src="https://github.com/user-attachments/assets/63e11f6c-c68b-4288-bfad-b6d732668d14"> <br> This will tell the ESP to broadcast the extended pedal info. <br>
3) Activate the "Log pedal states" switch <br> <img width="200" alt="Bildschirmfoto 2024-11-01 um 20 37 07" src="https://github.com/user-attachments/assets/91138b5d-9903-4736-b46f-159eefd1b0e8"> <br>
4) The pedal state record can be found in you Simhub installation directory, e.g.:  <br> <img width="500" alt="Bildschirmfoto 2024-11-01 um 20 37 07" src="https://github.com/user-attachments/assets/f32f461a-e1af-4a49-a92b-64daa4e745e4">

# Visualize the pedal logs
1) Open [jupyter online](https://python-fiddle.com/)
2) Upload the plotting code by opening "upload code context menu" <br>
<img width="400" alt="Bildschirmfoto 2024-11-01 um 20 37 07" src="https://github.com/user-attachments/assets/878bfa80-b024-4f1e-bb99-c0760aeb9418"> <br>
and inserting the [jupyther notebook](https://github.com/ChrGri/DIY-Sim-Racing-FFB-Pedal/blob/develop/Validation/VisualizePedalLog.ipynb) link <br>
<img width="400" alt="Bildschirmfoto 2024-11-01 um 20 37 07" src="https://github.com/user-attachments/assets/c6f1538d-eec2-43c0-b3dd-773b0bd40678">  <br>

3) Upload the pedal log file <br> <img width="400" alt="Bildschirmfoto 2024-11-01 um 20 44 36" src="https://github.com/user-attachments/assets/2b86cc91-302d-4b39-808c-e154702d0e92">

4) Change the pedal log file name accordingly<br> <img width="400" alt="Bildschirmfoto 2024-11-01 um 20 45 49" src="https://github.com/user-attachments/assets/0ab3f6b1-0a0b-494d-9e6e-b352df4c3c08">

5) Run the script <br> <img width="400" alt="Bildschirmfoto 2024-11-01 um 20 46 36" src="https://github.com/user-attachments/assets/76f00347-1ef6-413e-806a-3c908aa736a3">

6) Inspect the generated graphs, e.g. <br> <img width="980" alt="Bildschirmfoto 2024-11-01 um 20 48 59" src="https://github.com/user-attachments/assets/d8d3c60a-5d7f-44c2-a404-40fc3253edc1">
