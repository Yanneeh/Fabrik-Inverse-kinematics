import numpy as np
import math
import sys
import os
import time
import matplotlib as mpl
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
from mpl_toolkits.mplot3d import axes3d

def unit(vector):

  # Non-library functie.
  # mag = abs(math.sqrt(math.pow(vector[0], 2) + math.pow(vector[1], 2)))
  # print(mag)

  # Bereken de magnitude van de vector
  mag = np.linalg.norm(vector)

  # Bereken de unit vector doormiddel van de oude vector door de magnitude te delen.
  unit = vector / mag

  # Stuur de unit vector terug.
  return unit

def angleBetween(vector1, vector2):
  # Bereken de hoek tussen twee vectoren. Bereken de inverse cosinus van de twee input vectoren keer elkaar en gedeeld door de magnitude van de twee input vectoren keer elkaar. Convert deze hoek daarna naar graden.
  angle = math.degrees(math.acos( (vector1[0] * vector2[0] + vector1[1] * vector2[1]) / (np.linalg.norm(vector1) * np.linalg.norm(vector2)) ))

  # Stuur de hoek terug.
  return angle

def getAngle(vector1, vector2=np.array([0, 0])):
  # Bereken de hoek tussen een vector en de x-as. Bereken de inverse tangens van dy/dx en convert de hoek in radialen naar een hoek in graden.
  angle = math.degrees(math.atan2(vector1[1] - vector2[1], vector1[0] - vector2[0]))

  # Stuur de hoek terug.
  return angle

class Segment:
  """ Een onderdeel van de arm. """
  def __init__(self, x, y, lengte, angle):

    # Onthoudt de gegeven hoek.
    self.angle = angle

    # Onthoudt de gegeven lengte.
    self.lengte = lengte

    # Convert graden naar radian om de berekening uit te voeren.
    # Bereken de x waarde aan de hand van de gegeven hoek.
    dx = math.cos(math.radians(angle)) * lengte

    # Bereken de y waarde aan de hand van de gegeven hoek.
    dy = math.sin(math.radians(angle)) * lengte

    # Maak een nieuwe vector op basis van de oude vector.
    self.v = np.array([x + dx, y + dy])

class Arm:
  """ De eerste versie van de arm class. Aan de arm kunnen nieuwe segmenten worden toegevoegd. De arm moet ook naar een punt in ruimte kunnen bewegen. """
  def __init__(self, x=0, y=0):

    # Initialiseer beginpunt als vector.
    self.beginpoint = np.array([x, y])

    # Initialiseer segments list.
    self.segments = []

    # De totale lengte van de hele arm.
    self.armlengte = 0

    # De z-as van de robot.
    self.zAngle = 0

    # Het einpunt van de robot. Op dit moment leeg.
    self.endpoint = np.array([False, False])

  """ Voeg een nieuw segment toe aan de keten. Geef bij deze functie de lengte van de segment en welke hoek hij heeft. """
  def addSegment(self, length, angle):

    # Kijk of de eerste segment al geinitialiseerd is. Zo ja, bouw verder. Zo nee, maak dan een begin segment.
    if len(self.segments) > 0:

      # Maak een segment van de vector van de laatste segment, de nieuwe lengte en de nieuwe hoek.

      # Tel ook de parent angle erbij op voor realisme.
      segment = Segment(self.segments[-1].v[0], self.segments[-1].v[1], length, angle + self.segments[-1].angle)
    else:

      # Maak een segment van de vector beginpoint, lengte en hoek.
      segment = Segment(self.beginpoint[0], self.beginpoint[1], length, angle)

    # Voeg lengte toe aan de totale armlengte.
    self.armlengte = self.armlengte + segment.lengte

    # Voeg de nieuwe segment toe aan de list.
    self.segments.append(segment)

  def calc2D(self, x, y):
    self.endpoint = np.array([x, y])

    if len(self.segments) > 1:

        # Check of de afstand tussen het eindpunt en het beginpunt kleiner is dan de totale lengte van de arm.
        if np.linalg.norm(self.beginpoint - self.endpoint) < self.armlengte:

            # Fabrik algoritme.
            while np.linalg.norm(self.segments[-1].v - self.endpoint) > 0.1:
                # Achteruit. EXPERIMENT: (Ik zou de lengte van -1 naar -2 kunnen veranderen).
                for i in range(len(self.segments) - 1, 0, -1):

                  # Op het uiteinde moeten we eerst het eindpunt gebruiken om de formule te kunnen toepassen.

                  # Kijk of de waarde van i gelijk is aan de index van de laatse vector aan de arm.
                  if i == len(self.segments) - 1:
                    # Ga nog een index lager naar de een na laatse vector in de list. Gebruik dan de formule met de eindvector en vermenigvuldig met de lengte van de vector met de laatste index.
                    vector = (unit(self.segments[i-1].v - self.endpoint) * self.segments[i].lengte) + self.endpoint
                    # print(vector)

                    # Vervang oude vector met nieuwe vector.
                    self.segments[i-1].v = vector
                    # print(i-1)

                    # plt.plot([vector[0]], [vector[1]], 'go')
                    # plt.text(vector[0], vector[1] + 1, 'Vector')
                  else:
                    vector = (unit(self.segments[i-1].v - self.segments[i].v) * self.segments[i].lengte) + self.segments[i].v
                    # print(vector)
                    # print(i-1)

                    self.segments[i-1].v = vector

                    # plt.plot([vector[0]], [vector[1]], 'go')
                    # plt.text(vector[0], vector[1] + 1, 'Vector')

                # Vooruit.
                for i in range(len(self.segments)):
                  # print(i)
                  if i == 0:
                    vector = (unit(self.segments[i].v - self.beginpoint) * self.segments[i].lengte) + self.beginpoint
                    # print(vector)

                    self.segments[i].v = vector

                    # plt.plot([vector[0]], [vector[1]], 'yo')
                    # plt.text(vector[0], vector[1] + 1, 'Vector')

                  elif i == len(self.segments) - 1:
                    vector = (unit(self.segments[i-1].v - self.endpoint) * self.segments[i].lengte * -1) + self.segments[i-1].v
                    # print(vector)

                    self.segments[i].v = vector

                    # plt.plot([vector[0]], [vector[1]], 'bo')
                    # plt.text(vector[0], vector[1] + 1, 'Vector')
                  else:
                    vector = (unit(self.segments[i].v - self.segments[i-1].v) * self.segments[i].lengte) + self.segments[i-1].v
                    # print(vector)

                    self.segments[i].v = vector

                    # plt.plot([vector[0]], [vector[1]], 'yo')
                    # plt.text(vector[0], vector[1] + 1, 'Vector')

            # Bereken nieuwe hoeken.
            for i in range(len(self.segments)):
                if i == 0:
                  # Bereken de hoek tussen de eerste vector in de list en de x as.
                  angle = getAngle(self.segments[i].v)

                  print(angle)

                  # Update de hoek van deze vector.
                  self.segments[i].angle = angle
                else:
                  # Reken de hoek tussen 2 vectoren uit. Gebruik hier de huidige en vorige index van de list.
                  angleBeteenVector = angleBetween(self.segments[i].v, self.segments[i-1].v)

                  angle = math.degrees(math.asin( (np.linalg.norm(self.segments[i].v) * math.sin(math.radians(angleBeteenVector))) / self.segments[i].lengte))

                  print(angle)

                  # Update de hoek van deze vector.
                  self.segments[i].angle = angle

        else:
          print('Point too far...')
          sys.exit()
    else:
        print('Add segments first... A minimum of 2 segments is required.')
        sys.exit()

  def calc3D(self, x, y, z):
    self.endpoint = np.array([x, y])

    # Check of de afstand tussen het eindpunt en het beginpunt kleiner is dan de totale lengte van de arm.
    if np.linalg.norm(self.beginpoint - self.endpoint) < self.armlengte:
        while np.linalg.norm(self.segments[-1].v - self.endpoint) > 0.1:

            # Achteruit. EXPERIMENT: (Ik zou de lengte van -1 naar -2 kunnen veranderen).
            for i in range(len(self.segments) - 1, 0, -1):

              # Op het uiteinde moeten we eerst het eindpunt gebruiken om de formule te kunnen toepassen.

              # Kijk of de waarde van i gelijk is aan de index van de laatse vector aan de arm.
              if i == len(self.segments) - 1:
                # Ga nog een index lager naar de een na laatse vector in de list. Gebruik dan de formule met de eindvector en vermenigvuldig met de lengte van de vector met de laatste index.
                vector = (unit(self.segments[i-1].v - self.endpoint) * self.segments[i].lengte) + self.endpoint
                # print(vector)

                # Vervang oude vector met nieuwe vector.
                self.segments[i-1].v = vector
                # print(i-1)

                # plt.plot([vector[0]], [vector[1]], 'go')
                # plt.text(vector[0], vector[1] + 1, 'Vector')
              else:
                vector = (unit(self.segments[i-1].v - self.segments[i].v) * self.segments[i].lengte) + self.segments[i].v
                # print(vector)
                # print(i-1)

                self.segments[i-1].v = vector

                # plt.plot([vector[0]], [vector[1]], 'go')
                # plt.text(vector[0], vector[1] + 1, 'Vector')

            # Vooruit.
            for i in range(len(self.segments)):
              if i == 0:
                vector = (unit(self.segments[i].v - self.beginpoint) * self.segments[i].lengte) + self.beginpoint
                # print(vector)

                self.segments[i].v = vector

                # plt.plot([vector[0]], [vector[1]], 'yo')
                # plt.text(vector[0], vector[1] + 1, 'Vector')

              elif i == len(self.segments) - 1:
                vector = (unit(self.segments[i-1].v - self.endpoint) * self.segments[i].lengte * -1) + self.segments[i-1].v
                # print(vector)

                self.segments[i].v = vector

                # plt.plot([vector[0]], [vector[1]], 'bo')
                # plt.text(vector[0], vector[1] + 1, 'Vector')
              else:
                vector = (unit(self.segments[i].v - self.segments[i-1].v) * self.segments[i].lengte) + self.segments[i-1].v
                # print(vector)

                self.segments[i].v = vector

                # plt.plot([vector[0]], [vector[1]], 'yo')
                # plt.text(vector[0], vector[1] + 1, 'Vector')
        # Bereken nieuwe hoeken.
        for i in range(len(self.segments)):
            if i == 0:
              # Bereken de hoek tussen de eerste vector in de list en de x as.
              angle = getAngle(self.segments[i].v, self.beginpoint)

              print(angle)

              # Update de hoek van deze vector.
              self.segments[i].angle = angle
            else:
              # Reken de hoek tussen 2 vectoren uit. Gebruik hier de huidige en vorige index van de list.
              angleBeteenVector = angleBetween(self.segments[i].v, self.segments[i-1].v)

              angle = math.degrees(math.asin( (np.linalg.norm(self.segments[i].v) * math.sin(math.radians(angleBeteenVector))) / self.segments[i].lengte))

              print(angle)

              # Update de hoek van deze vector.
              self.segments[i].angle = angle

        # Bereken de z-hoek van de robot.
        zPos = np.array([self.segments[-1].v[0], z])
        zAngle = getAngle(zPos)

        self.zAngle = zAngle

        print(zAngle)

    else:
      print('Point too far...')
      sys.exit()

  def plt2D(self, save=False, name="graph"):
      # Plot arm.
      for i in range(len(self.segments)):
        plt.plot([self.segments[i].v[0]], [self.segments[i].v[1]], 'ro')
        plt.text(self.segments[i].v[0], self.segments[i].v[1] + 1, '(x:{}, y:{})'.format(int(self.segments[i].v[0]), int(self.segments[i].v[1])))

      # Startpunt
      plt.plot([self.beginpoint[0]], [self.beginpoint[1]], 'bo')
      plt.text(self.beginpoint[0], self.beginpoint[1], 'Startpunt')

      if np.any(self.endpoint):
          # Eindpunt
          plt.plot([self.endpoint[0]], [self.endpoint[1]], 'yo')
          plt.text(self.endpoint[0], self.endpoint[1] + 20, 'Eindpunt')

      plt.axis([-600, 600, -600, 600])
      plt.grid(True)

      if save == True:
          plt.savefig('{}.png'.format(name))

      plt.show(block=True)

  def plt3D(self):
      # Plot in 3D.
      fig = plt.figure()
      ax1 = fig.add_subplot(111, projection="3d")

      # Plot arm.
      for segment in self.segments:
          ax1.scatter(z, segment.v[0], segment.v[1], c='r')
          # plt.text(segment.v[0], segment.v[1] + 1, '(x:{}, y:{})'.format(int(segment.v[0]), int(segment.v[1])))

      # Startpunt
      ax1.scatter(z, self.beginpoint[0], self.beginpoint[1])

      # Eindpunt
      ax1.scatter(z, self.endpoint[0], self.endpoint[1], c='g')

      ax1.set_xlabel('z-axis')
      ax1.set_ylabel('x-axis')
      ax1.set_zlabel('y-axis')

      plt.show()

  def sendPoints():
      pass
