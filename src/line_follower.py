#!/usr/bin/python3

import rospy
import cv2
import numpy as np
import time

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped


class LineFollower:

    def __init__(self, robot_name):

        # ROS Node starten
        rospy.init_node('line_follower', anonymous=True)

        # speichern das neuste Kamerabild
        self.image = None

        # Duckiebot startet im normalen Fahrmodus
        self.state = "FOLLOWING"
        self.last_correction = 0.0

        # Speed Challenge
        # Startzeit merken, damit wir später die Rundenzeit berechnen können
        self.start_time = time.monotonic()
        self.race_finished = False

        # CvBridge um ROS-Bilder mit OpenCV zu verarbeiten
        self.bridge = CvBridge()

        # Unsere Hauptschleife läuft mit 10 Hz
        self.rate = rospy.Rate(10)

        # Kamera Subscriber
        # Wir empfangen die komprimierten Kamerabilder vom Duckiebot
        camera_topic = '/' + robot_name + '/camera_node/image/compressed'

        rospy.Subscriber(
            camera_topic,
            CompressedImage,
            self.callback
        )

        # Wheel Publisher
        # Über dieses Topic schicken wir die Geschwindigkeiten an die Räder
        wheel_topic = '/' + robot_name + '/wheels_driver_node/wheels_cmd'

        self.wheel_pub = rospy.Publisher(
            wheel_topic,
            WheelsCmdStamped,
            queue_size=1
        )

        # Räder beim Beenden stoppen
        rospy.on_shutdown(self.stop_wheels)

        print("================================")
        print("SPEED CHALLENGE GESTARTET!")
        print("Timer läuft.")
        print("================================")


    def stop_wheels(self):

        # Beide Räder auf 0 setzen
        # Diese Funktion verwenden wir bei Rot, Linienverlust, Ziel und Shutdown
        msg = WheelsCmdStamped()
        msg.vel_left = 0.0
        msg.vel_right = 0.0

        self.wheel_pub.publish(msg)


    def callback(self, data):

        # Bei jedem neuen Kamerabild speichern wir die neuste Nachricht
        # Die Verarbeitung machen wir erst in process_image()
        self.image = data


    def process_image(self):

        # Nach dem Zieleinlauf keine Fahrbefehle mehr senden
        if self.race_finished:
            self.stop_wheels()
            return

        # Solange noch kein Kamerabild angekommen ist, können wir nichts verarbeiten
        if self.image is None:
            return

        # ROS Image -> OpenCV Image
        # Die Kamera liefert komprimierte Bilder, OpenCV braucht aber ein normales Bild
        img = self.bridge.compressed_imgmsg_to_cv2(
            self.image,
            "bgr8"
        )

        # Bildhöhe und Bildbreite auslesen
        height, width, channels = img.shape

        # Gesamtes Kamerabild in HSV umwandeln
        # Mit HSV können wir bestimmte Farben einfacher herausfiltern
        hsv_full = cv2.cvtColor(
            img,
            cv2.COLOR_BGR2HSV
        )


        # 1. GRÜN ERKENNEN
        # HSV-Bereich für Grün festlegen
        green_lower_hsv = np.array([40, 70, 70])
        green_upper_hsv = np.array([85, 255, 255])

        # Alle grünen Pixel werden in der Maske weiss, der Rest schwarz
        green_mask = cv2.inRange(
            hsv_full,
            green_lower_hsv,
            green_upper_hsv
        )

        # Grüne Pixel zählen
        # Ab 10000 Pixeln erkennen wir genügend Grün für die Ampel
        green_pixels = cv2.countNonZero(green_mask)
        green_threshold = 3000


        # 2. ROT ERKENNEN
        # Rot liegt im HSV-Farbraum in zwei Bereichen
        # Deshalb brauchen wir zwei Masken für die rote Farbe
        red_lower_1 = np.array([0, 100, 100])
        red_upper_1 = np.array([10, 255, 255])

        red_lower_2 = np.array([170, 100, 100])
        red_upper_2 = np.array([179, 255, 255])

        red_mask_1 = cv2.inRange(
            hsv_full,
            red_lower_1,
            red_upper_1
        )

        red_mask_2 = cv2.inRange(
            hsv_full,
            red_lower_2,
            red_upper_2
        )

        # Beide roten Bereiche zu einer Maske zusammenführen
        red_mask = cv2.bitwise_or(
            red_mask_1,
            red_mask_2
        )

        # Ab 15000 roten Pixeln soll der Duckiebot stoppen
        red_pixels = cv2.countNonZero(red_mask)
        red_threshold = 15000


        # 3. BLAUES ZIEL ERKENNEN
        # Dunkelblau / Navy / Blauviolett
        # Die HSV-Grenzen haben wir erweitert, damit auch dunkles Blau erkannt wird
        blue_lower_hsv = np.array([100, 60, 35])
        blue_upper_hsv = np.array([155, 255, 255])

        blue_mask = cv2.inRange(
            hsv_full,
            blue_lower_hsv,
            blue_upper_hsv
        )

        # Wir verwenden einen höheren Schwellenwert,
        # damit nicht schon kleine blaue Flächen das Rennen beenden
        blue_pixels = cv2.countNonZero(blue_mask)
        blue_threshold = 40000

        # Debug-Ausgabe, um zu sehen, wie viele Pixel pro Farbe erkannt werden
        print(
            "FARBEN:",
            "GRUEN =", green_pixels,
            "BLAU =", blue_pixels,
            "ROT =", red_pixels
        )


        # Blau hat Priorität vor der Ampelsteuerung
        # Das Ziel wird nur erkannt, wenn genügend Blau vorhanden ist
        # und gleichzeitig nicht zu viel Rot oder Grün erkannt wird
        # So vermeiden wir, dass eine Ampel fälschlicherweise als Ziel zählt
        if (blue_pixels > blue_threshold
            and red_pixels < red_threshold
            and green_pixels < green_threshold
        ):

            # Rennen als beendet markieren
            self.race_finished = True

            # Vergangene Zeit seit dem Programmstart berechnen
            elapsed_time = time.monotonic() - self.start_time

            print("================================")
            print("ZIEL ERREICHT!")
            print("Rundenzeit:", round(elapsed_time, 2), "Sekunden")
            print("================================")

            # Nach dem Ziel sofort stoppen
            self.stop_wheels()
            return


        # 4. AMPELSTEUERUNG
        # Wenn wir fahren und Rot erkennen, wechseln wir in den Wartezustand
        if self.state == "FOLLOWING" and red_pixels > red_threshold:

            self.state = "WAITING"
            print("ROT ERKANNT -> STOP")

        # Wenn wir warten, darf der Duckiebot erst bei Grün weiterfahren
        elif self.state == "WAITING" and green_pixels > green_threshold:

            self.state = "FOLLOWING"
            print("GRÜN ERKANNT -> WEITER")

        # Bei Rot stehen bleiben, bis Grün erkannt wird
        # Der Zustand bleibt gespeichert, auch wenn Rot nicht mehr sichtbar ist
        if self.state == "WAITING":

            self.stop_wheels()
            return


        # 5. GELBE LINIE ERKENNEN

        # Nur untere 40 % des Kamerabildes verwenden
        # Dort sehen wir hauptsächlich den Boden und die Linie vor dem Duckiebot
        roi_start = int(height * 0.6)
        roi = img[roi_start:, :]

        # BGR -> HSV
        hsv = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2HSV
        )

        # HSV-Bereich für die gelbe Linie
        yellow_lower_hsv = np.array([20, 100, 100])
        yellow_upper_hsv = np.array([35, 255, 255])

        # Gelbe Pixel aus dem unteren Bildbereich herausfiltern
        mask = cv2.inRange(
            hsv,
            yellow_lower_hsv,
            yellow_upper_hsv
        )

        # Anzahl gelber Pixel bestimmen
        yellow_pixels = cv2.countNonZero(mask)

        # Mit den Bildmomenten können wir den Schwerpunkt der Linie berechnen
        moments = cv2.moments(mask)

        # Falls keine gelbe Linie erkannt wird, stoppen wir
        # So vermeiden wir auch eine Division durch 0 bei der Schwerpunktberechnung
        if moments["m00"] == 0:
            print("LINIE VERLOREN")
            self.stop_wheels()
            return

        # Schwerpunkt der gelben Linie
        # m10 / m00 ergibt die horizontale Position der erkannten gelben Fläche
        cx = int(moments["m10"] / moments["m00"])

        # Mitte des Kamerabildes bestimmen
        center_x = width // 2

        # Abweichung zwischen Linie und Bildmitte berechnen
        # Positiver Fehler = Linie rechts
        # Negativer Fehler = Linie links
        error = cx - center_x

        # Fehler normalisieren, damit er ungefähr zwischen -1 und 1 liegt
        # Dadurch hängt die Steuerung weniger von der Bildauflösung ab
        error_normalized = error / (width / 2)


        # 6. P-CONTROLLER - GERADE + 90-GRAD-KURVEN
        # Kleine Abweichungen ignorieren
        # Damit der Duckiebot nicht wegen jedem kleinen Fehler korrigiert
        if abs(error_normalized) < 0.04:
            error_normalized = 0.0

        # Bei grossen Abweichungen langsamer fahren und stärker lenken
        # Das brauchen wir vor allem für die scharfen Kurven
        if abs(error_normalized) > 0.35:
            # STARKE KURVE
            base_speed = 0.06
            kp = 0.25
            max_correction = 0.10

        else:
            # GERADE STRECKE
            # Hier reicht eine kleinere Korrektur, damit der Bot ruhiger fährt
            base_speed = 0.10
            kp = 0.08
            max_correction = 0.025

        # Direkte Lenkkorrektur ohne Glättung
        # kp bestimmt, wie stark wir auf den aktuellen Fehler reagieren
        correction = kp * error_normalized

        # Korrektur begrenzen, damit die Lenkung nicht zu stark wird
        correction = np.clip(
            correction,
            -max_correction,
            max_correction
        )

        # Radgeschwindigkeiten berechnen
        # Ist die Linie rechts, fährt das linke Rad schneller -> Bot lenkt rechts
        # Ist die Linie links, fährt das rechte Rad schneller -> Bot lenkt links
        vel_left = base_speed + correction
        vel_right = base_speed - correction

        # Debug-Ausgabe, um den Fehler und die Radgeschwindigkeiten zu kontrollieren
        print(
            "Linie:", cx,
            "Fehler:", round(error_normalized, 2),
            "Links:", round(vel_left, 2),
            "Rechts:", round(vel_right, 2)
        )

        # Geschwindigkeiten in eine ROS-Nachricht schreiben
        msg = WheelsCmdStamped()
        msg.vel_left = vel_left
        msg.vel_right = vel_right

        # Radbefehle an den Duckiebot senden
        self.wheel_pub.publish(msg)


    def run(self):

        # Hauptschleife läuft, solange ROS aktiv ist
        while not rospy.is_shutdown():

            # Neustes Kamerabild verarbeiten und Radbefehle berechnen
            self.process_image()

            # Frequenz auf ungefähr 10 Durchläufe pro Sekunde begrenzen
            self.rate.sleep()


if __name__ == '__main__':

    # Unser Duckiebot heisst omega
    # Hier erstellen wir den LineFollower und starten die Hauptschleife
    line_follower = LineFollower('omega')
    line_follower.run()