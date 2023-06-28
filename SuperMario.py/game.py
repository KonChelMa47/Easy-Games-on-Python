import pygame
import sys
import time
from pygame import mixer
import random

class Background:
    def __init__(self, x, y, img_path):
        self.x = x
        self.y = y
        self.img = pygame.transform.scale(pygame.image.load(img_path), (800, 600))

    def move_left(self, speed):
        self.x -= speed

    def move_right(self, speed):
        self.x += speed

    def draw(self, screen):
        screen.blit(self.img, (self.x, self.y))
        text_font = pygame.font.Font(None, 36)
        text_surface = text_font.render(f"Go Go Go HAMANAKA", True, (255, 255, 255))
        screen.blit(text_surface, (0, 0))

    def update(self):
        if self.x <= -800:
            self.x = 800


class Block:
    def __init__(self, x, y, img_path):
        self.x = x
        self.y = y
        self.img = pygame.transform.scale(pygame.image.load(img_path), (200, 20))

    def draw(self, screen):
        screen.blit(self.img, (self.x, self.y))

    def get_rect(self):
        return self.img.get_rect().move(self.x, self.y)
    
    def move_left(self, speed):
        self.x -= speed

class Item:
    def __init__(self, x, y, img_path):
        self.x = x
        self.y = y
        self.img = pygame.transform.scale(pygame.image.load(img_path), (90, 90))
        self.get = False
        self.gravity = 0.2
        self.velocity = 0
        self.on_ground = False

    def draw(self, screen):
        screen.blit(self.img, (self.x, self.y))

    def get_rect(self):
        return self.img.get_rect().move(self.x, self.y)

    def move_left(self, speed):
        self.x -= speed

    def move_right(self, speed):
        self.x += speed

    def update(self, player_rect, blocks):
        self.velocity += self.gravity
        self.y += self.velocity

        if self.y >= 500:
            self.y = 500
            self.velocity = 0
            self.on_ground = True
        else:
            self.on_ground = False

        item_rect = self.img.get_rect().move(self.x, self.y)
        if item_rect.colliderect(player_rect):
            self.x = -1000
            self.get = True

        for block in blocks:
            block_rect = block.get_rect()
            if item_rect.colliderect(block_rect) and self.velocity >= 0:
                
                self.y = block.y - self.img.get_height()
                self.velocity = 0
                self.on_ground = True
                break
        

class Fill:
    def __init__(self, x, y, img_path):
        self.x = x
        self.y = y
        self.img = pygame.transform.scale(pygame.image.load(img_path), (90, 90))

    def draw(self, screen):
        screen.blit(self.img, (self.x, self.y))

class Get:
    def __init__(self, x, y, img_path):
        self.x = x
        self.y = y
        self.img = pygame.transform.scale(pygame.image.load(img_path), (90, 90))

    def draw(self, screen):
        screen.blit(self.img, (self.x, self.y))

class Boss:
    def __init__(self, x, y, img_path):
        self.x = x
        self.y = y
        self.img = pygame.transform.scale(pygame.image.load(img_path),(200, 200))
        self.img1 = pygame.transform.scale(pygame.image.load("motoki2.png"),(200, 200))
        self.img2 = pygame.transform.scale(pygame.image.load("motoki3.png"),(200, 200))
        self.gravity = 0.2
        self.velocity = 0
        self.on_ground = False
        self.defeated = False
        self.direction = 1

    def draw(self, screen, count):
        if self.defeated:
            self.x = -1000
        elif count == 1:
            screen.blit(self.img1, (self.x, self.y))
        elif count == 2:
            screen.blit(self.img2, (self.x, self.y))
        elif count == 0:
            screen.blit(self.img, (self.x, self.y))

    def jump(self):
        if self.on_ground:
            self.velocity = -2

    def get_rect(self):
        return self.img.get_rect().move(self.x, self.y)
        

    def selfmove_left(self, enemyspeed):
        self.x -= enemyspeed

    def selfmove_right(self, enemyspeed):
        self.x += enemyspeed

    def update(self, blocks, motokispeed):
        self.velocity += self.gravity
        self.y += self.velocity

        if self.y >= 400:
            self.y = 400
            self.velocity = 0
            self.on_ground = True
        else:
            self.on_ground = False
            
        enemy_rect = self.img.get_rect().move(self.x, self.y)
        for block in blocks:
            block_rect = block.get_rect()
            if enemy_rect.colliderect(block_rect) and self.velocity >= 0:
                
                self.y = block.y - self.img.get_height()
                self.velocity = 0
                self.on_ground = True
                break

        if self.direction == 1:
            self.move_right(motokispeed)
            if self.x >= 600:
                self.direction = -1
        else:
            self.move_left(motokispeed)
            if self.x <= 10:
                self.direction = 1

    def move_left(self, bgspeed):
        self.x -= bgspeed

    def move_right(self, bgspeed):
        self.x += bgspeed


class Enemy:
    def __init__(self, x, y, img_path):
        self.x = x
        self.y = y
        self.img = pygame.transform.scale(pygame.image.load(img_path),(120, 150))
        self.gravity = 0.2
        self.velocity = 0
        self.on_ground = False
        self.defeated = False
        self.direction = 1

    def draw(self, screen):
        if self.defeated:
            self.x = -1000
        else:
            screen.blit(self.img, (self.x, self.y))

    def jump(self):
        if self.on_ground:
            self.velocity = -8

    def get_rect(self):
        return self.img.get_rect().move(self.x, self.y)
        

    def selfmove_left(self, enemyspeed):
        self.x -= enemyspeed

    def selfmove_right(self, enemyspeed):
        self.x += enemyspeed

    def update(self, blocks, enemyspeed):
        self.velocity += self.gravity
        self.y += self.velocity

        if self.y >= 450:
            self.y = 450
            self.velocity = 0
            self.on_ground = True
        else:
            self.on_ground = False
            
        enemy_rect = self.img.get_rect().move(self.x, self.y)
        for block in blocks:
            block_rect = block.get_rect()
            if enemy_rect.colliderect(block_rect) and self.velocity >= 0:
                
                self.y = block.y - self.img.get_height()
                self.velocity = 0
                self.on_ground = True
                break

        if self.direction == 1:
            self.move_right(enemyspeed)
            if self.x >= 600:
                self.direction = -1
        else:
            self.move_left(enemyspeed)
            if self.x <= 10:
                self.direction = 1

    def move_left(self, bgspeed):
        self.x -= bgspeed

    def move_right(self, bgspeed):
        self.x += bgspeed

class Player:
    def __init__(self, x, y, img_path):
        self.x = x
        self.y = y
        self.img_normal = pygame.transform.scale(pygame.image.load(img_path), (100, 124))
        self.img_hit = pygame.transform.scale(pygame.image.load("playerfall.png"), (100, 124))
        self.gravity = 0.15
        self.velocity = 0
        self.on_ground = False
        self.hit = False

    def draw(self, screen):
        if self.hit:
            screen.blit(self.img_hit, (self.x, self.y))
        else:
            screen.blit(self.img_normal, (self.x, self.y))

    def jump(self):
        if self.on_ground:
            self.velocity = -8

    def get_rect(self):
        return self.img_normal.get_rect().move(self.x, self.y)

    def move_left(self, speed):
        self.x -= speed

    def move_right(self, speed):
        self.x += speed

    def update(self, blocks, enemies):
        self.velocity += self.gravity
        self.y += self.velocity

        if self.y >= 450:
            self.y = 450
            self.velocity = 0
            self.on_ground = True
        else:
            self.on_ground = False


        player_rect = self.img_normal.get_rect().move(self.x, self.y)

        for block in blocks:
            block_rect = block.get_rect()
            if player_rect.colliderect(block_rect):
                if self.velocity > 0 and player_rect.bottom <= block_rect.top + 10:
                    # プレイヤーが上からブロックに接触した場合
                    self.y = block.y - self.img_normal.get_height()
                    self.velocity = 0
                    self.on_ground = True
                elif self.velocity < 0 and player_rect.top >= block_rect.bottom - 10:
                    # プレイヤーが下からブロックに接触した場合
                    self.y = block.y + block_rect.height
                    self.velocity = 0
                elif self.velocity >= 0 and player_rect.right > block_rect.left and player_rect.left < block_rect.right:
                    # プレイヤーが横からブロックに接触した場合
                    if player_rect.bottom > block_rect.top + 10:
                        self.y = block.y - self.img_normal.get_height()
                        self.velocity = 0
                        self.on_ground = True
                    else:
                        if player_rect.left < block_rect.left:
                        # プレイヤーがブロックの左側に接触している場合
                            self.x = block.x - self.img_normal.get_width()
                            self.velocity = 0
                        else:
                        # プレイヤーがブロックの右側に接触している場合
                            self.x = block.x + block_rect.width
                            self.velocity = 0
        
                    

    def take_hit(self):
        if not self.hit:
            self.hit = True

def countdown(seconds):
    while seconds > 0:
        start_time = time.time()
        while time.time() - start_time < 1:
            pass
        seconds -= 1
            

def main():
    global music1
    global music2
    global gameover

    pygame.init()
    pygame.display.set_caption("HAMANAKA ADVENTURE")
    width = 800
    height = 600
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()
    gameover_image = pygame.image.load("gameover.jpg")
    gameover_image = pygame.transform.scale(gameover_image, (800, 600))

    gameclear_image = pygame.image.load("gameclear.jpg")
    gameclear_image = pygame.transform.scale(gameclear_image, (800, 600))

    bg1 = Background(0, 0, 'bg2.jpg')
    bg2 = Background(800, 0, 'bg3.jpg')
    bg3 = Background(1600, 0, "bg3.jpg")

    player = Player(10, 450, 'mario.png.PNG')

    simakazu = Enemy(700, 400, "simakazu.png")
    natsume = Enemy(-1000, 100, "natsume.png")
    kaunapawa = Enemy(-1000, 200, "kaunapawa.png")
    motoki = Boss(-1000, 200, "motoki.png")


    item1 = Item(-1000, 300, "kakomon.png")
    item2 = Item(-1100, 400, "kakomon.png")
    item3 = Item(-1100, 400, "kakomon.png")
    item4 = Item(-1100, 400, "kakomon.png")

    get1 = Get(700, 0, "kakomon.png")
    get2 = Get(600, 0 , "kakomon.png")
    get3 = Get(500, 0 , "kakomon.png")
    get4 = Get(400, 0 , "kakomon.png")

    fills = [Fill(700, 0 , "kakomonwaku.png"), Fill(600, 0 , "kakomonwaku.png"), Fill(500, 0 , "kakomonwaku.png"), Fill(400, 0 , "kakomonwaku.png")]

    blocks = [Block(0, 400, "platform.png"), Block(300 ,200, "platform.png"), Block(600 ,400, "platform.png"), Block(300, 590, "block.jpg"), Block(1000 , 450, "platform.png"), Block(1200 , 300, "platform.png"), Block(1400 , 100, "platform.png"), Block(1600 , 300, "platform.png"), Block(1400, 590, "block.jpg"), Block(1800, 450, "platform.png"), Block(2000, 550, "platform.png"), Block(2300 ,200, "platform.png"), Block(2600 ,400, "platform.png"), Block(2300, 590, "block.jpg"), Block(2900 ,500, "platform.png"), Block(3000 ,300, "platform.png"), Block(3200 ,100, "platform.png"), Block(3500 ,600, "platform.png"), Block(3700, 400, "platform.png"), Block(3900 ,200, "platform.png"), Block(4200 ,400, "platform.png"), Block(3900, 590, "block.jpg"), Block(4100, 400, "platform.png"), Block(4500 ,200, "platform.png"), Block(4600 ,400, "platform.png"), Block(4900, 590, "block.jpg"), Block(5100, 590, "block.jpg"), Block(5300, 590, "block.jpg"), Block(4800 , 450, "platform.png"), Block(5500, 590, "block.jpg"), Block(5700, 590, "block.jpg"), Block(5900, 590, "block.jpg"), Block(6100, 590, "block.jpg"), Block(6300, 590, "block.jpg"), Block(6500, 590, "block.jpg"), Block(6700, 590, "block.jpg")]

    music1 = pygame.mixer.Sound("gustygalaxy.mp3")
    music2 = pygame.mixer.Sound("gameover.mp3")
    music3 = pygame.mixer.Sound("scream.mp3")
    music4 = pygame.mixer.Sound("boom.mp3")
    music5 = pygame.mixer.Sound("gameclear.mp3")
    music6 = pygame.mixer.Sound("screammotoki.mp3")
    music7 = pygame.mixer.Sound("gun.mp3")
    music1.play()

    death = pygame.image.load("playerfall.png")
    death = pygame.transform.scale(death, (200, 400))
    
    speed = 5
    enemyspeed = 1
    bgspeed = 0.5
    motokispeed = 1

    jumping = False

    gameover = False
    gameclear = False

    count = 0

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    jumping = True

            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_SPACE:
                    jumping = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_g:
                    countdown(10)

        player_rect = player.get_rect()
 
        simakazu_rect = simakazu.get_rect()
        natsume_rect = natsume.get_rect()
        kaunapawa_rect = kaunapawa.get_rect()
        motoki_rect = motoki.get_rect()

        if player_rect.colliderect(simakazu_rect):
            if player.velocity > 0 and player_rect.bottom <= simakazu_rect.top + 10:
                player.velocity = -8
                music3.play()
                player.hit = False
                simakazu.defeated = True
                item1.x = 300
                natsume.x = 3000

            else:
                player.take_hit()
                music1.stop()
                music2.play()
                player.hit = True
                gameover = True

        if player_rect.colliderect(natsume_rect):
            if player.velocity > 0 and player_rect.bottom <= natsume_rect.top + 10:
                player.velocity = -8
                music3.play()
                player.hit = False
                natsume.defeated = True
                item2.x = 300
                kaunapawa.x = 2500

            else:
                player.take_hit()
                music1.stop()
                music2.play()
                player.hit = True
                gameover = True

        if player_rect.colliderect(kaunapawa_rect):
            if player.velocity > 0 and player_rect.bottom <= kaunapawa_rect.top + 10:
                player.velocity = -8
                music3.play()
                player.hit = False
                kaunapawa.defeated = True
                item3.x = 300
                motoki.x = 2000

            else:
                player.take_hit()
                music1.stop()
                music2.play()
                player.hit = True
                gameover = True

        if player_rect.colliderect(motoki_rect):
            if player.velocity > 0 and player_rect.bottom <= motoki_rect.top + 10:
                player.velocity = -10
                music4.play()
                player.hit = False
                count += 1
                motokispeed += 5
                motoki.jump()


            else:
                player.take_hit()
                music1.stop()
                music2.play()
                player.hit = True
                gameover = True

        if count >= 2:
            music7.play()

        if count == 3:
            music4.play()
            motoki.defeated = True
            item4.x = 300

        if item4.get:
            music1.stop()
            music5.play()
            gameclear = True
                
        if gameclear:
            screen.blit(gameclear_image, (0, 0))
            pygame.display.update()
            countdown(12)
            pygame.quit()
            sys.exit()

        if gameover:
            screen.blit(gameover_image, (0, 0))
            screen.blit(death, (500, 200))
            pygame.display.update()
            countdown(5)
            pygame.quit()
            sys.exit()

        if jumping:
            player.jump()
            
        keys = pygame.key.get_pressed()

        if keys[pygame.K_LEFT]:
            if not player.x <= 0:
                player.move_left(speed)

        elif keys[pygame.K_RIGHT]:

            if player.x < 320:
                player.move_right(speed)

            elif not simakazu.defeated:
                player.move_right(speed)

            elif simakazu.defeated:

                bg1.move_left(bgspeed)
                bg2.move_left(bgspeed)
                bg3.move_left(bgspeed)

                natsume.move_left(speed)
                kaunapawa.move_left(speed)
                motoki.move_left(speed)

                item1.move_left(speed)
                item2.move_left(speed)
                item3.move_left(speed)
                item4.move_left(speed)

                for block in blocks:
                    block.move_left(speed)


        bg1.update()
        bg2.update()
        bg3.update()

        player.update(blocks, simakazu)

        item1.update(player_rect, blocks)
        item2.update(player_rect, blocks)
        item3.update(player_rect, blocks)
        item4.update(player_rect, blocks)

        simakazu.update(blocks, enemyspeed)
        natsume.update(blocks, enemyspeed)
        kaunapawa.update(blocks, enemyspeed)
        motoki.update(blocks, motokispeed)

        bg1.draw(screen)
        bg2.draw(screen)
        bg3.draw(screen)

        player.draw(screen)

        simakazu.draw(screen)

        if simakazu.defeated:
            natsume.draw(screen)

        if natsume.defeated:
            kaunapawa.draw(screen)

        if kaunapawa.defeated:
            motoki.draw(screen, count)

        for block in blocks:
            block.draw(screen)

        for fill in fills:
            fill.draw(screen)

        if simakazu.defeated:
            item1.draw(screen)

        if natsume.defeated:
            item2.draw(screen)

        if kaunapawa.defeated:
            item3.draw(screen)

        if motoki.defeated:
            item4.draw(screen)
        
        if item1.get:
            get1.draw(screen)

        if item2.get:
            get2.draw(screen)

        if item3.get:
            get3.draw(screen)

        if item4.get:
            get4.draw(screen)

        pygame.display.update()

        clock.tick(60)


if __name__ == "__main__":
    main()