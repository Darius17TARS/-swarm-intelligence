import math
from math import sin, cos, atan2, radians, degrees
from random import randint
import pygame as py

'''
Swarm Intelligence - https://github.com/Darius17TARS/-swarm-intelligence
'''




bg = py.image.load("C:\\Users\\dariu\\Desktop\\BOIDS\\images.jpg")
start_time = py.time.get_ticks()
BGCOLOR = (0, 0, 161)

		

class Schwarm(py.sprite.Sprite):
    

    def __init__(self, SchwarmSurface):
        super().__init__()
        global lerp_color
        global start_time
        self.SchwarmSurface = SchwarmSurface
        self.image = py.Surface((10, 10))
        SchwarmColor = (236, 242, 253)
    
        

        
        py.draw.circle(self.image, SchwarmColor, (5,5), 5)
        self.pSpace = (self.image.get_width() + self.image.get_height()) / 2
        self.direction = py.Vector2(1, 0)  
        dS_w, dS_h = self.SchwarmSurface.get_size()
        self.rect = self.image.get_rect(center=(randint(50, dS_w - 50), randint(50, dS_h - 50)))
        self.angle = randint(0, 360)  
        self.pos = py.Vector2(self.rect.center)

    def update(self, allBoids, dt, ejWrap=False):   # The mathematics were imported from another public project of Nikolaus Stromberg
        selfCenter = py.Vector2(self.rect.center)
        curW, curH = self.SchwarmSurface.get_size()
        turnDir = xvt = yvt = yat = xat = 0
        turnRate = 120 * dt
        margin = 48
        neiboids = sorted([         # gets list of nearby boids, sorted by distance
            iBoid for iBoid in allBoids
            if py.Vector2(iBoid.rect.center).distance_to(selfCenter) < self.pSpace*12 and iBoid != self ],
            key=lambda i: py.Vector2(i.rect.center).distance_to(selfCenter)) # 200
        del neiboids[7:]  # keep 7 closest, dump the rest
        if (ncount := len(neiboids)) > 1:  # when boid has neighborS (walrus sets ncount)
            nearestBoid = py.Vector2(neiboids[0].rect.center)
            for nBoid in neiboids:  # adds up neighbor vectors & angles for averaging
                xvt += nBoid.rect.centerx
                yvt += nBoid.rect.centery
                yat += sin(radians(nBoid.angle))
                xat += cos(radians(nBoid.angle))
            tAvejAng = degrees(atan2(yat, xat)) #round()
            targetV = (xvt / ncount, yvt / ncount)
            # if too close, move away from closest neighbor
            if selfCenter.distance_to(nearestBoid) < self.pSpace : targetV = nearestBoid
            tDiff = targetV - selfCenter  # get angle differences for steering
            tDistance, tAngle = py.math.Vector2.as_polar(tDiff)
            # if boid is close enough to neighbors, match their average angle
            if tDistance < self.pSpace*6 : tAngle = tAvejAng # and ncount > 2
            # computes the difference to reach target angle, for smooth steering
            angleDiff = (tAngle - self.angle) + 180
            if abs(tAngle - self.angle) > .8: turnDir = (angleDiff / 360 - (angleDiff // 360)) * 360 - 180
            # if boid gets too close to target, steer away
            if tDistance < self.pSpace and targetV == nearestBoid : turnDir = -turnDir
        # Avoid edges of screen by turning toward the edge normal-angle
        if not ejWrap and min(self.pos.x, self.pos.y, curW - self.pos.x, curH - self.pos.y) < margin:
            if self.pos.x < margin : tAngle = 0
            elif self.pos.x > curW - margin : tAngle = 180
            if self.pos.y < margin : tAngle = 90
            elif self.pos.y > curH - margin : tAngle = 270
            angleDiff = (tAngle - self.angle) + 180
            turnDir = (angleDiff / 360 - (angleDiff // 360)) * 360 - 180
            edgeDist = min(self.pos.x, self.pos.y, curW - self.pos.x, curH - self.pos.y)
            turnRate = turnRate + (1 - edgeDist / margin) * (20 - turnRate) #minRate+(1-dist/margin)*(maxRate-minRate)
        if turnDir != 0:  # steers based on turnDir, handles left or right
            self.angle += turnRate * abs(turnDir) / turnDir
            self.angle %= 360  # ensures that the angle stays within 0-360
        # adjusts angle of boid image to match heading
        self.rect = self.image.get_rect(center=self.rect.center)  # recentering fix
        self.direction = py.Vector2(1, 0).rotate(self.angle).normalize()
        next_pos = self.pos + self.direction * (180 + (7-ncount)**2) * dt #(3.5 + (7-ncount)/14) * (fps * dt)
        self.pos = next_pos
        # optional screen wrap
        if ejWrap and not self.SchwarmSurface.get_rect().contains(self.rect):
            if self.rect.bottom < 0 : self.pos.y = curH
            elif self.rect.top > curH : self.pos.y = 0
            if self.rect.right < 0 : self.pos.x = curW
            elif self.rect.left > curW : self.pos.x = 0
        # actually update position of boid
        self.rect.center = self.pos

def main():
    
    py.init()  
    py.display.set_caption("SchwarmIntelligenz")
    
    screen = py.display.set_mode((1500, 900), py.RESIZABLE)
    nBoids = py.sprite.Group()
    for n in range(100):
        nBoids.add(Schwarm(screen))
    allBoids = nBoids.sprites()
    clock = py.time.Clock()

    while True:
        for e in py.event.get():
            if e.type == py.QUIT:
                return

        dt = clock.tick(60) / 1000
        screen.fill(BGCOLOR)
        nBoids.update(allBoids, dt, False)
        nBoids.draw(screen)
        py.display.update()

if __name__ == '__main__':
    main()  
    py.quit()
