import random

def Name():
    global coin
    global count
    global Name
    global tut
    
    count = 0
    coin = 100
    Name = input("あなたの名前を入力してください")
    print("Yo!",Name,)
    print("俺はこのパチンコ店の店長ドミニクだ")
    print("さっそく始めよう！")
    print("------------------------------------------")

Name()

def ent():
    global tut
    tut = input("チューリアルをはじめますか？y/n")
    if tut == "y":
        print("はじめるか")
        print("------------------------------------------")
        Tutorial()
    elif tut == "n":
        print("はじめてじゃないのね")
        print("------------------------------------------")
        Loop()
    else:
        print("yかnで答えろ")
        print("------------------------------------------")
        ent()

def Tutorial():
    coin = 100
    print("今回はお前に",coin,"だけコインをやる")
    print("君みたいな初心者には十分の額だ")
    input()
    print("そして今から",Name,"のために特別にルール説明してやる")
    input()
    print(Name,"のすべきことはただbet数を入力しスロット回すことだけだ")
    print("簡単なことだが頭をつかってかけないとすぐになくなるぞ！")
    input()
    print("そして勝てばかけたbetは5倍で返ってくる！")
    print("だが負けたら！かけた分だけなくなるから注意しろ")
    input()
    print("5回勝てばゲームは終わりスコアが出てくる")
    input()
    print("説明はこれだけだ")
    print("ではさっそくはじめるぞー")
    input()
    print("------------------------------------------")
    Loop()

def Kake():
    
    global bet
    
    
    while 1:
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
        
    elif 0 < bet / coin * 100 <= 19:
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

ent()
    
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
        return True
        
    else:
        return False
  
       
while 1:
    if Pachinko() == True:
        count += 1
        coin += bet * 4
        if count == 5:
            print("------------------------------------------") 
            print("あなたのコイン数は",coin,"です。")
            print("------------------------------------------")
            if 500 <= coin:
                print("やられたよ！この店の鍵を",Name,"様にやるよ")
                
            elif 300 <= coin < 500:
                print(Name,"さん！あなたは真のパチンカスだ！すばらしい")
                
            elif 200 < coin < 300:
                print("感動したよー",Name,"、お前にはポテンシャルあるぞ")
                
            elif 100 <= coin < 200:
                print("マイナスにならなかっただけマシや！",Name,"、お前はがんばった")
                
            elif 50 <= coin < 100:
                print("マイナスじゃーん！はぁー？",Name,"、おめえ向いてないな")
                
            else:
                print("下手すぎだろ！一生ギャンブルすな！",Name,"!")
            
            print("You Did It!")
                
            break
        else:
            pass
        print("やるねぇ～その調子")
        print("あなたのコイン数は",coin,"です。")
        print("------------------------------------------")
        
        Loop()

    else:
        coin -= bet
        if coin <= 0:
            print("------------------------------------------")
            print("二度と俺の店に来るな！！働け！！")
            print("------------------------------------------")
            print("ゲームオーバー")
            break
        else:
            pass
        print("おっとっと、外れてるよ～")
        print("あなたのコイン数は残り",coin,"です。")
        print("------------------------------------------")
        
        Loop()
