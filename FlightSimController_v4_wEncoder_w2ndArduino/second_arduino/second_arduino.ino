int RButton1=2; // düğmeyi 6 nolu dijital pine bağlamıştık
int RButton2=3;
void setup() {
  Serial.begin(9600); // 9600 hızında Seri iletişimi başlatıyoruz
pinMode(RButton1, INPUT); // 2 nolu pinden sinyal okuyacağımızı belirttik
pinMode(RButton2, INPUT);
}

void loop() {
int deger= digitalRead(RButton1); // dugmeye basıldığında degere 1 aktarılcak
if(deger==1){ 
  Serial.write(1); // düğmeye basılırsa seri yoldan 1 sayısını yolluyoruz
}else{
  Serial.write(0);// düğmeye basılmazsa 0 sayısını yolluyoruz.
  }
}
