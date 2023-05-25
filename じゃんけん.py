import random

x = input("あなたの手は？[グー、チョキ、パー」")
print("あなたの手は",x,"です。")

random.seed()

y = random.randint(1, 3)

if (y ==1):
    print("\n私は[チョキ]です")

if (y == 2):
    print("\n私は[グー]です")
    
if (y == 3):
    print("\n私は[パー]です")
    
if y == 1 and x =="グー":
    print("\nあなたの「勝ちです」")
    
if y == 2 and x =="グー":
    print("\n[引き分け]です")

if y == 3 and x =="グー":
    print("\nあなたの「負け」です")
    
if y == 1 and x =="チョキ":
    print("\n[引き分け]です")
    
if y == 2 and x =="チョキ":
    print("\nあなたの「負け」です")

if y == 3 and x =="チョキ":
    print("\nあなたの「勝ち」です")
    
if y == 1 and x =="パー":
    print("\nあなたの「負け」です")
    
if y == 2 and x =="パー":
    print("\nあなたの「勝ち」です")

if y == 3 and x =="パー":
    print("\n[引き分け]です")
