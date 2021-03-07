#ifndef INC_WINDOWED_AVERAGE_H_
#define INC_WINDOWED_AVERAGE_H_

typedef struct WindowedAverage {
	int16_t data_[40];
	int16_t oldest_;
	int16_t newest_;
	int32_t total_;
	size_t capacity_;
	size_t size_;
	size_t pos_;
} WindowedAverage;

void WindowedAverage_Init(WindowedAverage* wa) {
	wa->total_ = 0;
	wa->capacity_ = 40;
	wa->size_ = 0;
	wa->pos_ = 0;
	wa->oldest_ = 0;
	wa->newest_ = 0;
}

void WindowedAverage_Add(WindowedAverage* wa, int16_t value) {
	const int16_t old = wa->data_[wa->pos_];
	wa->oldest_ = old;
	wa->newest_ = value;
	wa->total_ += value;
	wa->data_[wa->pos_] = value;
	wa->pos_ = (wa->pos_ + 1) % wa->capacity_;
	size_t old_size = wa->size_;
	wa->size_ = min(wa->size_ + 1, wa->capacity_);

	if (old_size == wa->size_) {
	  wa->total_ -= old;
	}
}

#endif /* INC_WINDOWED_AVERAGE_H_ */
