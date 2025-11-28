# IoT_AccionamientoMotorElectrico

Accionamiento de un motor Dahlander con interconexión y control IoT.

Contexto: 
- Control del motor de una mezcladora con dos velocidades para permitir agregar nuevo material al recipiente cuando se detecta con el sensor de proximidad inductivo. 

Propiedades: 
- Página web para control remoto que permita: arranque en ambos sentidos, cambio de velocidad, parada.
- Conexión con broquer mqtt para detectar: subida de temperatura --> parar el motor; sensor de proximidad --> disminuir velocidad y cambio de sentido.
- Conexionado eléctrico que disminuya la velocidad del motor cuando detecte un metal el sensor de proximidad.
