## image size 210 width by 299 height a millimeter grid is had.
## 60 mm across 80 mm up
## to skip the cut on a line place a black pixel in the first column 
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import csv

fpi = 'tst.png'
fpo = 'out.png'

toolprfl = [4, 4, 4, 4, 5, 7, 9, 11, 12, 14, 15, 17, 19, 20, 22, 24, 25, 27, 29, 30, 32, 34, 35, 37, 38, 40, 42, 43, 45, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47]

cimg = mpimg.imread(fpi)
img = np.zeros((299,210), dtype = int)
omtrx = np.zeros((299,210,3), dtype = int)

dpth = list()
cls = list()
clstf = list()
for i in range(cimg.shape[0]):
    for j in range(cimg.shape[1]):
        img[i,j] = int(85*(cimg[i,j,0] + cimg[i,j,1] + cimg[i,j,2]))

for i in range(img.shape[0]):
    find = False
    for j in range(img.shape[1]):
        if img[i,j] < 128:
            dpth.append(j)
            find = True
            break
    if find == False:
        dpth.append(210)        
        
for i in range(0,299):
    bcls = False
    for j in range (0,89):
        if i-j > 0 and dpth[i] != 0:
            if dpth[i] - dpth[i-j] >= toolprfl[j] - 1 and dpth[i-j] != 0:
                bcls = True

    if bcls == True:
        cls.append(i)
    
    clstf.append(bcls)
    
print()       
print("Tool Colisions at:")
print(cls)

for i in range(299):
    for j in range(210):
        if dpth[i] > j and dpth[i] != 0:
            if clstf[i] == True:
                omtrx[i,j,0] = 255
                omtrx[i,j,1] = 200
                omtrx[i,j,2] = 200
            else:
                omtrx[i,j,:] = 255
        else:
            if clstf[i] == True:
                omtrx[i,j,0] = 200
                omtrx[i,j,1] = 0 
                omtrx[i,j,2] = 0 
            else:
                omtrx[i,j,:] = 0

        if dpth[i] == 0:
            omtrx[i,j,0] = 200
            omtrx[i,j,1] = 255
            omtrx[i,j,2] = 200
    
plt.imsave(fpo, omtrx, cmap = 'gray')


with open('instrctln.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=' ',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
    for i in range(299):
        writer.writerow([dpth[i]])
with open('instrct.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, dialect = 'excel')
    writer.writerow([dpth])
    
    
if cls != []:
    print("Tool Colisions Exist in Output File; Procede with Caution")