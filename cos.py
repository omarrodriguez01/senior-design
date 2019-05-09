import math
def checkValidity(a, b, c):  
      
    # check condition  
    if (a + b <= c) or (a + c <= b) or (b + c <= a) : 
        return False
    else: 
        return True  

def calY(a,b,c):
    if checkValidity(a, b, c): 
        print("Valid")  
    else: 
        print("Invalid")
        return

        
    top = (math.pow(a,2)+math.pow(b,2)-math.pow(c,2))
    bottom = (2.0 * a * b )
    aC = math.acos(top/bottom)
    aB = math.asin((b*math.sin(aC))/c)
    aA = math.asin((a*math.sin(aB))/b)

    if (aB > math.radians(90)):
        aB = math.radians(180)-aB
    aY = math.radians(90) - aB
    print math.degrees(aY)

    y = math.sin(aY)*a
    print y
    return y

  
# driver code  
c = 4.0
a = 13.0
b = 26.0






y = calY(a,b,c)
x = math.sqrt(math.pow(a,2)-math.pow(y,2))
print x
