import re 
from word2number import w2n
import nltk

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
    return ''