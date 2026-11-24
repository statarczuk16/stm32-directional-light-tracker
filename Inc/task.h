struct task {
	void (*run)(void);
	uint32_t interval_ms;
	uint32_t last_ms;
};
