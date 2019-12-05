import re 
from word2number import w2n
import nltk
nltk.download('punkt')

def convertToNum(data):
    tokens = nltk.word_tokenize(data)
    tokensP = []    
    for token in tokens:
        try:
            tokensP.append(w2n.word_to_num(token))
        except:
            pass
    data = "".join(str(tokensP))
    data = unicode(data, 'utf-8')
    result = list([val for val in data if val.isnumeric() or val.isspace()]) 
    return result[0]
