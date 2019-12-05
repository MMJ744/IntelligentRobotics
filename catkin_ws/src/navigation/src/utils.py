import re 
from word2number import w2n
import nltk

numwords = {}
units = [
    "zero", "one", "two", "three", "four", "five", "six", "seven", "eight",
    "nine", "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen",
    "sixteen", "seventeen", "eighteen", "nineteen",
]
tens = ["", "", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety"]

scales = ["hundred", "thousand", "million", "billion", "trillion"]
def convertToNum(data):
    print(data)
    tokens = nltk.word_tokenize(str(data))
    tokensP = []    
    print(tokens)
    for token in tokens:
        try:
            tokensP.append(w2n.word_to_num(token))
        except:
            if token in ['1','2','3','4','5','6','7','8','9']:
                tokensP.append(token)
    print(tokensP)
    data = "".join(str(tokensP))
    data = unicode(data, 'utf-8')
    print(data)
    result = list([val for val in data if val.isnumeric() or val.isspace()]) 
    print(result)
    if len(result) == 0: return ''
    print(result[0])
    return result[0]

def getYesNo(data):
    tokens = data.split()
    if 'yes' in tokens:
        return 'yes'
    if 'no' in tokens:
        return 'no'
    return None

def getNum(data):
    tokens = data.split()
    reg = '[0-9]+'
    for x in tokens:
        m = re.match(reg,x)
        if m:
            print(m.group(0))
            return m.group(0)
    analyise = []
    started = False
    keepgoing = True
    for x in tokens:
        if x in units or x in tens or x in scales:
            if not started or keepgoing:
                started = True
                analyise.append(x)
        elif started:
            keepgoing = False
    if not analyise == []:
        return text2int(analyise)
    return ''

def text2int(textnum):
    global numwords
    global units
    global tens
    global scales
    if numwords=={}:
      numwords["and"] = (1, 0)
      for idx, word in enumerate(units):    numwords[word] = (1, idx)
      for idx, word in enumerate(tens):     numwords[word] = (1, idx * 10)
      for idx, word in enumerate(scales):   numwords[word] = (10 ** (idx * 3 or 2), 0)

    current = result = 0
    for word in textnum:
        if word not in numwords:
            print('text2int error')
            return ''
        scale, increment = numwords[word]
        current = current * scale + increment
        if scale > 100:
            result += current
            current = 0

    return result + current
print(getNum("There are twenty four people in my group"))
