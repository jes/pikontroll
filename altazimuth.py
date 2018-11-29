# altazimuth calculations for pikontroll
# based on c code from http://pikon.patrickaalto.com/pikonblog.html
# jes 2018

import math
import time

# return current unix time in microseconds
def GetNow():
    return time.time() * 1000000

# https://answers.yahoo.com/question/index?qid=20080630193348AAh4zNZ
# return current "local sidereal degrees"
def CalculateLocalSiderealDegrees(longitude, utcNow):
    D = (utcNow - 946728000 * 1000000) / (1000000.0*60.0*60.0*24.0)
    return math.fmod(280.461+360.98564737 * D + longitude, 360.0)

# return (azimuthDegrees, altitudeDegrees)
def ComputeAltAzimuth(utcNow, longitudeDegrees, latitudeDegrees, rightAscensionHours, declinationDegrees):
    localSiderealDegrees = CalculateLocalSiderealDegrees(longitudeDegrees, utcNow)
    rightAscensionDegrees = rightAscensionHours * 15.0;  # Convert hours to degrees by multiplying with 15
    hourAngleDegrees = localSiderealDegrees - rightAscensionDegrees
    if hourAngleDegrees < 0.0:
        hourAngleDegrees += 360.0
    hourAngleRadians = 2.0 * math.pi / 360.0 * hourAngleDegrees
    declinationRadians = 2.0 * math.pi / 360.0 * declinationDegrees
    latitudeRadians = 2.0 * math.pi / 360.0 * latitudeDegrees

    altitudeRadians = math.asin(math.sin(declinationRadians) * math.sin(latitudeRadians) + math.cos(declinationRadians) * math.cos(latitudeRadians) * math.cos(hourAngleRadians))
    azimuthRadians = math.acos((math.sin(declinationRadians) - math.sin(altitudeRadians) * math.sin(latitudeRadians)) / (math.cos(altitudeRadians) * math.cos(latitudeRadians)))

    altitudeDegrees = 360.0 / (2 * math.pi) * altitudeRadians
    azimuthDegrees = 360.0 / (2 * math.pi) * azimuthRadians

    if math.sin(hourAngleRadians) > 0:
        azimuthDegrees = 360.0 - azimuthDegrees

    return (azimuthDegrees, altitudeDegrees)

# return (rightAscensionHours, declinationDegrees)
def ComputeRaDec(utcNow, longitudeDegrees, latitudeDegrees, azimuthDegrees, altitudeDegrees):
    localSiderealDegrees = CalculateLocalSiderealDegrees(longitudeDegrees, utcNow)
    latitudeRadians = 2.0 * math.pi / 360.0 * latitudeDegrees
    altitudeRadians = 2.0 * math.pi / 360.0 * altitudeDegrees
    azimuthRadians = 2.0 * math.pi / 360.0 * azimuthDegrees
    declinationRadians = math.asin(math.sin(latitudeRadians) * math.sin(altitudeRadians) + math.cos(latitudeRadians) * math.cos(altitudeRadians) * math.cos(azimuthRadians))
    hourAngleRadians = math.acos((math.sin(altitudeRadians) - math.sin(declinationRadians) * math.sin(latitudeRadians))/(math.cos(declinationRadians) * math.cos(latitudeRadians)))
    hourAngleDegrees = 360.0 / (2.0 * math.pi) * hourAngleRadians
    rightAscensionDegrees = localSiderealDegrees - hourAngleDegrees
    if rightAscensionDegrees < 0.0:
        rightAscensionDegrees += 360.0
    rightAscensionHours = rightAscensionDegrees / 15.0
    declinationDegrees = 360.0 / (2.0 * math.pi) * declinationRadian
    return (rightAscensionHours, declinationDegrees)
