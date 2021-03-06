# i n c l u d e   < m l l . h >  
 u 8   r a n d o m _ f o r _ s 8 ( )  
 {  
     r e t u r n   r a n d ( ) % ( 2 5 5 ) - 1 2 7 ;  
 }  
  
 u 8   r a n d o m _ f o r _ f l o a t ( )  
 {  
     r e t u r n   2 / ( 1 + p o w ( 1 . 0 5 ,   r a n d ( ) % 1 0 0 ) )   -   1 ;  
 }  
  
 f l o a t   L o g i s t i c ( f l o a t   x )  
 {  
                 r e t u r n   1 / ( 1 + p o w ( 2 ,   - x ) ) ;  
 }  
 v o i d   i n i t _ w e i g h t s ( )  
 {  
         i n t   i ,   j ;  
 	  
         f o r ( i   =   0 ;   i   <   p i x e l _ n u m ;   i + + )  
 	 	 f o r ( j   =   0 ;   j   <   l a y e r 0 _ n o d e ;   j + + )  
 	 	 	 w e i g h t s 0 [ i ] [ j ]   =   r a n d o m _ f o r _ s 8 ( ) ;  
 	  
         f o r ( i   =   0 ;   i   <   l a y e r 0 _ n o d e ;   i + + )  
 	 	 f o r ( j   =   0 ;   j   <   l a y e r 1 _ n o d e ;   j + + )  
 	 	 	 w e i g h t s 1 [ i ] [ j ]   =   r a n d o m _ f o r _ f l o a t ( ) ;  
  
 	 	 f o r ( i   =   0 ;   i   <   l a y e r 1 _ n o d e ;   i + + )  
 	 	 f o r ( j   =   0 ;   j   <   o u t p u t _ n o d e ;   j + + )  
 	 	 	 w e i g h t s 2 [ i ] [ j ]   =   r a n d o m _ f o r _ f l o a t ( ) ;  
         / / p r i n t f ( " � � � � � � � � � �   d o n e \ n " ) ;  
 }  
  
 u 8   m l f o r w a r d ( )  
 {  
                 i n t   i ,   j ;  
  
 	 	 	 	 f o r ( j   =   0 ;   j   <   l a y e r 0 _ n o d e ;   j + + )  
                 {  
 	 	 	 	 	 	 	 	 f o r ( i   =   0 ;   i   <   p i x e l _ n u m ;   i + + )  
                                       l a y e r 0 [ j ]   + =   p i x e l [ i ] * ( ( s 3 2 ) w e i g h t s 0 [ i ] [ j ] ) ;  
                                 l a y e r 0 [ j ]   + =   l a y e r 0 _ s h i f t [ j ] ;  
 	 	 	 	 	 	 	 	 l a y e r 0 [ j ]   =   L o g i s t i c ( l a y e r 0 [ j ] ) ;  
                 }  
  
 	 	 	 	 f o r ( j   =   0 ;   j   <   l a y e r 1 _ n o d e ;   j + + )  
                 {  
 	 	 	 	 	 	 	 	 f o r ( i   =   0 ;   i   <   l a y e r 0 _ n o d e ;   i + + )  
                                       l a y e r 1 [ j ]   + =   l a y e r 0 [ i ] * w e i g h t s 1 [ i ] [ j ] ;  
                                 l a y e r 1 [ j ]   + =   l a y e r 1 _ s h i f t [ j ] ;  
 	 	 	 	 	 	 	 	 l a y e r 1 [ j ]   =   L o g i s t i c ( l a y e r 1 [ j ] ) ;  
                 }  
 	 	 	 	  
 	 	 	 	 f o r ( j   =   0 ;   j   <   o u t p u t _ n o d e ;   j + + )  
                 {  
 	 	 	 	 	 	 	 	 f o r ( i   =   0 ;   i   <   l a y e r 1 _ n o d e ;   i + + )  
                                       o u t p u t [ j ]   + =   l a y e r 1 [ i ] * w e i g h t s 2 [ i ] [ j ] ;  
                                 o u t p u t [ j ]   + =   o u t p u t _ s h i f t [ j ] ;  
 	 	 	 	 	 	 	 	 o u t p u t [ j ]   =   L o g i s t i c ( o u t p u t [ j ] ) ;  
                 }  
  
                 i f ( o u t p u t [ 0 ] > = o u t p u t [ 1 ] )  
 	 	 	 	 	 r e t u r n   0 ;  
 	 	 	 	 e l s e  
 	 	 	 	 	 r e t u r n   1 ;  
  
 }  
  
 v o i d   m l b a c k ( i n t   x )  
 {  
   	 	 	 	 i n t   i ,   j ;  
 	  
 	 	 	 	 s 8   w e i g h t s 0 _ s h i f t [ p i x e l _ n u m ] [ l a y e r 0 _ n o d e ] ;  
 	 	 	 	 f l o a t   w e i g h t s 1 _ s h i f t [ l a y e r 0 _ n o d e ] [ l a y e r 1 _ n o d e ] ;  
 	 	 	 	 f l o a t   w e i g h t s 2 _ s h i f t [ l a y e r 1 _ n o d e ] [ o u t p u t _ n o d e ] ;  
 	  
 	 	 	 	 f l o a t   l a y e r 1 _ e [ l a y e r 1 _ n o d e ] ;  
 	 	 	 	 f l o a t   l a y e r 0 _ e [ l a y e r 0 _ n o d e ] ;  
 	  
                 / / p r i n t f ( " � � � � � � � �   =   % d \ n " ,   x ) ;  
                 f o r ( i   =   0 ;   i   <   o u t p u t _ n o d e ;   i + + )  
                 {  
                                 i f ( i   = =   x ) {  
                                                 f o r ( j   =   0 ;   j   <   l a y e r 1 _ n o d e ;   j + + )  
                                                                 { w e i g h t s 2 _ s h i f t [ j ] [ i ]   =   ( 1 - o u t p u t [ i ] ) * l a y e r 1 [ j ] ;  
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 l a y e r 1 _ e [ j ]   + =   ( 1 - o u t p u t [ i ] ) * w e i g h t s 2 [ j ] [ i ] ;  
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 }  
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 o u t p u t _ s h i f t [ i ]   + =   ( 1 - o u t p u t [ i ] ) * ( 1 - o u t p u t [ i ] ) * o u t p u t [ i ] ;  
                                 }  
                                 e l s e  
                                 {  
                                                 f o r ( j   =   0 ;   j   <   8 ;   j + + )  
                                                                 { w e i g h t s 2 _ s h i f t [ j ] [ i ]   =   ( - o u t p u t [ i ] ) * l a y e r 1 [ j ] ;  
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 l a y e r 1 _ e [ j ]   + =   ( - o u t p u t [ i ] ) * w e i g h t s 2 [ j ] [ i ] ;  
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 }  
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 o u t p u t _ s h i f t [ i ]   + =   ( - o u t p u t [ i ] ) * ( 1 - o u t p u t [ i ] ) * o u t p u t [ i ] ;  
                                 }  
                 }  
  
  
                 f o r ( i   =   0 ;   i   <   l a y e r 1 _ n o d e ;   i + + )  
 	 	 	 	 {  
                                 f o r ( j   =   0 ;   j   <   l a y e r 0 _ n o d e ;   j + + )  
                                                 { w e i g h t s 1 _ s h i f t [ j ] [ i ]   =   l a y e r 1 _ e [ i ] * l a y e r 0 [ j ] ;  
 	 	 	 	 	 	 	 	 	 	 	 	 l a y e r 0 _ e [ j ]   + =   l a y e r 1 _ e [ i ] * w e i g h t s 1 [ j ] [ i ] ;  
 	 	 	 	 	 	 	 	 	 	 	 	 }  
 	 	 	 	 	 	 	 	 l a y e r 1 _ s h i f t [ i ]   + =   l a y e r 1 _ e [ i ] * l a y e r 1 [ i ] * ( 1 - l a y e r 1 [ i ] ) ;  
 	 	 	 	 }  
 	 	 	 	  
 	 	 	 	 f o r ( i   =   0 ;   i   <   l a y e r 0 _ n o d e ;   i + + )  
 	 	 	 	 {  
                                 f o r ( j   =   0 ;   j   <   p i x e l _ n u m ;   j + + )  
                                                 { w e i g h t s 0 _ s h i f t [ j ] [ i ]   =   ( s 8 ) ( l a y e r 0 _ e [ i ] * p i x e l [ j ] ) ;  
 	 	 	 	 	 	 	 	 	 	 	 	 }  
 	 	 	 	 	 	 	 	 l a y e r 0 _ s h i f t [ i ]   =   l a y e r 0 _ e [ i ] * l a y e r 0 [ i ] * ( 1 - l a y e r 0 [ i ] ) ;  
 	 	 	 	 }  
  
 	 	 	 	 f o r ( i   =   0 ;   i   <   p i x e l _ n u m ;   i + + )  
                                 f o r ( j   =   0 ;   j   <   l a y e r 0 _ n o d e ;   j + + )  
                                                 w e i g h t s 0 [ i ] [ j ]   + =   w e i g h t s 0 _ s h i f t [ i ] [ j ] ;  
  
 	 	 	 	 f o r ( i   =   0 ;   i   <   l a y e r 0 _ n o d e ;   i + + )  
                                 f o r ( j   =   0 ;   j   <   l a y e r 1 _ n o d e ;   j + + )  
                                                 w e i g h t s 1 [ i ] [ j ]   + =   w e i g h t s 1 _ s h i f t [ i ] [ j ] ;  
 	 	 	 	  
 	 	 	 	 f o r ( i   =   0 ;   i   <   l a y e r 1 _ n o d e ;   i + + )  
                                 f o r ( j   =   0 ;   j   <   o u t p u t _ n o d e ;   j + + )  
                                                 w e i g h t s 2 [ i ] [ j ]   + =   w e i g h t s 2 _ s h i f t [ i ] [ j ] ;  
  
 } 