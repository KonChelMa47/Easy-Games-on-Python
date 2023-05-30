import random

def Name():
    global coin
    Name = input("あなたの名前を入力してください")
    print("こんにちわ",Name,"さん")
    print("楽しい楽しいパチンコの世界へようこそ")
    print("さっそく始めよう！")
    coin = 100
    print("あなたのコイン数は",coin,"です")
    print("------------------------------------------")
    
Name()

def Kake():
    
    global bet
    
    
    while True:
        try:           
            bet = int(input("bet数を入力してください"))
            break
        
        except ValueError as e:
            print()
    
    if 50 <= bet / coin * 100 <= 100:
        print("やるねー、男やな")
        return True
        
    elif 20 <= bet / coin * 100 <= 49:
        print("男じゃねーな、まあがんばれ")
        return True
        
    elif 0 <= bet / coin * 100 <= 19:
        print("チキンやん、もっとかけろや")
        return True
    
    else :
        print("なんしとん？コイン数ちゃんと読んだ？")
        return False
    
def Loop():
    while 1:
        if Kake() == True:
            break
    
        else:
            pass

Loop()
    
print("------------------------------------------")

    

def Pachinko():
    
    a = random.randint(1, 2)    
    b = random.randint(1, 2)
    c = random.randint(1, 2)

    input("スロットスタート！")
    print(a)

    input()
    print(b)

    input()
    print(c)

    if a==b==c:
        print("You Win!!!")
        return True
        
    elif a==b or a==c or b==c:
        print("Keep Going!")
        return False
        
    else:
        print("You Lose:(")
        return False
  
       
while 1:
    if Pachinko() == True:
        coin += bet * 2
        print("あなたのコイン数は",coin,"です。")
        print("------------------------------------------")
        
        Loop()
        
    else:
        coin -= bet
        if coin <= 0:
            print("GAME OVER LOSER!!")
            break
        else:
            pass
        print("あなたのコイン数は残り",coin,"です。")
        print("------------------------------------------")
        
        Loop()
