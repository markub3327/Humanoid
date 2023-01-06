#include <Servo.h>

#define CLIP_VALUE 20

#define NUM_OF_LEGS  6
#define NUM_OF_ARMS  2

Servo legs[NUM_OF_LEGS];
Servo arms[NUM_OF_ARMS];

int pos[NUM_OF_LEGS+NUM_OF_ARMS];

int normalize_angle(int angle)
{
  if (angle > 180)    return 180;
  else if (angle < 0) return 0;
  else                return angle;
}

void get_action(Servo *s, int *actual, int size, int clip_val)
{
  for (int i = 0; i < size; i++)
  {
    auto deg = random(actual[i] - clip_val, actual[i] + clip_val);
    deg = normalize_angle(deg);

    s[i].write( deg );

    // store new position
    actual[i] = deg;
  }
}

void setup() {
  randomSeed(analogRead(0));

  delay(15000);

  for (int i = 0; i < NUM_OF_LEGS; i++)
  {
    legs[i].attach(2 + i);
    legs[i].write(90);
    pos[i] = 90;
  }

  for (int i = NUM_OF_LEGS; i < 8; i++)
  {
    arms[i - NUM_OF_LEGS].attach(2 + i);
    arms[i - NUM_OF_LEGS].write(90);
    pos[i] = 90;
  }
}

void loop() {
  get_action(legs, pos, NUM_OF_LEGS, CLIP_VALUE);
  get_action(arms, &pos[NUM_OF_LEGS], NUM_OF_ARMS, CLIP_VALUE);
  delay( 500 );
}