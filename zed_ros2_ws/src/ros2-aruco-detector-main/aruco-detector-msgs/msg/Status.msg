# queue_size is the size of the current queue holding unprocessed images
# received_images a counter that tells how many images have been received so far
# processed_images a counter that tells how many images have been processed so far
# image_duration how long did it take to process the las 100 images (average) (in mili-seconds)


int32 queue_size
uint64 received_images
uint64 processed_images
float64 image_duration

