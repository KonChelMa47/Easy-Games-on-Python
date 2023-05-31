import pygame
import sys
from pygame import mixer


class Background:
    def __init__(self, x, y, img_path):
        self.x = x
        self.y = y
        self.img = pygame.transform.scale(pygame.image.load(img_path), (800, 600))

    def move_left(self, speed):
        self.x -= speed

    def draw(self, screen):
        screen.blit(self.img, (self.x, self.y))

    def update(self):
        if self.x <= -800:
            self.x = 800


class Block:
    def __init__(self, x, y, img_path):
        self.x = x
        self.y = y
        self.img = pygame.transform.scale(pygame.image.load(img_path), (50, 50))

    def draw(self, screen):
        screen.blit(self.img, (self.x, self.y))

    def get_rect(self):
        return self.img.get_rect().move(self.x, self.y)
    
    def move_left(self, speed):
        self.x -= speed


class Player:
    def __init__(self, x, y, img_path):
        self.x = x
        self.y = y
        self.img = pygame.transform.scale(pygame.image.load(img_path), (50, 62))
        self.gravity = 0.2
        self.velocity = 0
        self.on_ground = False

    def draw(self, screen):
        screen.blit(self.img, (self.x, self.y))

    def jump(self):
        if self.on_ground:
            self.velocity = -8

    def move_left(self, speed):
        self.x -= speed

    def move_right(self, speed):
        self.x += speed

    def update(self, blocks):
        self.velocity += self.gravity
        self.y += self.velocity

        if self.y >= 390:
            self.y = 390
            self.velocity = 0
            self.on_ground = True
        else:
            self.on_ground = False

        # プレイヤーとブロックの接触判定
        player_rect = self.img.get_rect().move(self.x, self.y)
        for block in blocks:
            block_rect = block.get_rect()
            if player_rect.colliderect(block_rect) and self.velocity >= 0:
                # プレイヤーがブロックの上に乗っている場合
                self.y = block.y - self.img.get_height()
                self.velocity = 0
                self.on_ground = True
                break


def main():
    pygame.init()
    pygame.display.set_caption("Super Mario Bros Py")
    screen = pygame.display.set_mode((800, 600))
    clock = pygame.time.Clock()

    bg1 = Background(0, 0, 'bgmario.png')
    bg2 = Background(800, 0, 'bgmario.png')
    player = Player(100, 390, 'mario.png.PNG')
    blocks = [Block(500, 300, "brickblock.png")]

    mixer.Sound('gustygalaxy.mp3').play()

    speed = 5

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    player.jump()

        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            player.move_left(speed)
        elif keys[pygame.K_RIGHT]:
            if player.x < 320:
                player.move_right(speed)
            else:
                bg1.move_left(speed)
                bg2.move_left(speed)
                for block in blocks:
                    block.move_left(speed)

        bg1.update()
        bg2.update()
        player.update(blocks)

        # Draw the backgrounds
        bg1.draw(screen)
        bg2.draw(screen)

        # Draw the player
        player.draw(screen)

        for block in blocks:
            block.draw(screen)

        # Update the display
        pygame.display.update()

        # Limit the frame rate
        clock.tick(60)


if __name__ == "__main__":
    main()
