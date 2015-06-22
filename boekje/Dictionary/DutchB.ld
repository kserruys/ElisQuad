[langid]
2067

[schemes]
0=/l10094/l


//***************************************** QWERTY **************************************
// Weight for spelling errors
// Categories:
//    one_letter_insert  - insertion of single letters
//    one_letter_delete  - removal of single letter
//    two_letters_change - change ordering of adjacent letters
//    one_letter_replace - letters substitution
//    split_word         - split word
// Weights:
//    1-9 - weight
// Threshold means default maximal sum of errors
[weights\0]
threshold=10
max_threshold=12
one_letter_insert=4
one_letter_delete=5
one_letter_replace=7
two_letters_change=2
split_word=8

// Substitutions
// Syntax:
//   <in_string>=<weight><out_string>,...
// Special strings:
//    . - given as in_string means any position in the word except for initial and final,
//        given as out_string means empty string
//    @ - means end of the word
//    # - means beginning of the word
[replacements\0]
a=6q,6w,5s,6z
b=5v,6g,6h,5n
c=5x,5d,5f,5v
d=5s,6e,6r,5f,6c,6x
e=5w,6s,6d,5r
f=5d,6r,6t,5g,6v,6c
g=5f,6t,6y,5h,6b,6v
h=5g,6y,6u,5j,6n,6b
i=5u,6j,6k,5o
j=5h,6u,6i,5k,6m,6n
k=5j,6i,6o,5l,6m
l=5k,6o,6p,5ç
m=5n,6j,6k
n=5b,6h,6j,5m
o=5i,5p,6l,6k
p=5o,6l,6ç
q=5w,6a
r=5e,5t,6f,6d
s=5a,6w,6e,5d,6x,6z
t=5r,5y,6g,6f
u=5y,5i,6j,6h
v=5c,6f,6g,5b
w=5q,5e,6s,6a
x=5z,6s,6d,5c
y=5t,5u,6h,6g
z=6a,6s,5x
ç=5l,6p

A=6Q,6W,5S,6Z
B=5V,6G,6H,5N
C=5X,5D,5F,5V
D=5S,6E,6R,5F,6C,6X
E=5W,6S,6D,5R
F=5D,6R,6T,5G,6V,6C
G=5F,6T,6Y,5H,6B,6V
H=5G,6Y,6U,5J,6N,6B
I=5U,6J,6K,5O
J=5H,6U,6I,5K,6M,6N
K=5J,6I,6O,5L,6M
L=5K,6O,6P,5Ç
M=5N,6J,6K
N=5B,6H,6J,5M
O=5I,5P,6L,6K
P=5O,6L,6Ç
Q=5W,6A
R=5E,5T,6F,6D
S=5A,6W,6E,5D,6X,6Z
T=5R,5Y,6G,6F
U=5Y,5I,6J,6H
V=5C,6F,6G,5B
W=5Q,5E,6S,6A
X=5Z,6S,6D,5C
Y=5T,5U,6H,6G
Z=6A,6S,5X

a=5A
b=5B
c=5C
d=5D
e=5E
f=5F
g=5G
h=5H
i=5I
j=5J
k=5K
l=5L
m=5M
n=5N
o=5O
p=5P
q=5Q
r=5R
s=5S
t=5T
u=5U
v=5V
w=5W
x=5X
y=5Y
z=5Z
ç=5Ç

A=5a
B=5b
C=5c
D=5d
E=5e
F=5f
G=5g
H=5h
I=5i
J=5j
K=5k
L=5l
M=5m
N=5n
O=5o
P=5p
Q=5q
R=5r
S=5s
T=5t
U=5u
V=5v
W=5w
X=5x
Y=5y
Z=5z

aa=3a
bb=3b
cc=3c
dd=3d
ee=3e
ff=3f
gg=3g
hh=3h
ii=3i
jj=3j
kk=3k
ll=3l
mm=3m
nn=3n
oo=3o
pp=3p
qq=3q
rr=3r
ss=3s
tt=3t
uu=3u
vv=3v
ww=3w
xx=3x
yy=3y
zz=3z
