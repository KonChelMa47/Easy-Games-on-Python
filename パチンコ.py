import random

a = random.randint(1, 7)
b = random.randint(1, 7)
c = random.randint(1, 7)

input("No.1 Enterをおしてください")
print(a)

input("No.2 Enterをおしてください")
print(b)

input("No.3 Enterをおしてください")
print(c)

if a==b==c:
    print("You Win!!!")
    
elif a==b or a==c or b==c:
    print("Keep Going!")
    
else:
    print("You Lose:(")
