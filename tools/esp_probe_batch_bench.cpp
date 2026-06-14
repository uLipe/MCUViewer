#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <unordered_map>
#include <vector>

#include "EspProbeSession.hpp"

static uint32_t parse_u32(const char* s)
{
	return static_cast<uint32_t>(std::strtoul(s, nullptr, 0));
}

static std::vector<std::pair<uint32_t, uint8_t>> build_entries(int argc, char* argv[], int first_addr_arg)
{
	std::vector<std::pair<uint32_t, uint8_t>> entries;
	for (int i = first_addr_arg; i < argc; ++i)
		entries.emplace_back(parse_u32(argv[i]), 4u);
	return entries;
}

static uint64_t bench_single(esp_probe::EspProbeSession& session, const std::vector<std::pair<uint32_t, uint8_t>>& entries, unsigned rounds)
{
	using clock = std::chrono::steady_clock;
	const auto t0 = clock::now();
	for (unsigned r = 0; r < rounds; ++r)
	{
		for (const auto& [address, size] : entries)
		{
			uint8_t buf[4]{};
			if (!session.readMemory(address, buf, size))
				return 0;
		}
	}
	const auto us = std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - t0).count();
	return static_cast<uint64_t>(us);
}

static uint64_t bench_batch(esp_probe::EspProbeSession& session, const std::vector<std::pair<uint32_t, uint8_t>>& entries, unsigned rounds)
{
	using clock = std::chrono::steady_clock;
	const auto t0 = clock::now();
	for (unsigned r = 0; r < rounds; ++r)
	{
		std::unordered_map<uint32_t, uint32_t> values;
		if (!session.readMemoryBatch(entries, values))
			return 0;
	}
	const auto us = std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - t0).count();
	return static_cast<uint64_t>(us);
}

int main(int argc, char* argv[])
{
	if (argc < 5)
	{
		std::fprintf(stderr, "usage: %s [chip] [speed_khz] [rounds] <addr_hex>...\n", argv[0]);
		return 1;
	}

	const char* chip = argv[1];
	const uint32_t speed_khz = parse_u32(argv[2]);
	const unsigned rounds = static_cast<unsigned>(parse_u32(argv[3]));
	const auto entries = build_entries(argc, argv, 4);
	if (entries.empty())
		return 1;

	esp_probe::EspProbeSession session;
	if (!session.connect("", chip, speed_khz))
	{
		std::fprintf(stderr, "connect failed: %s\n", session.lastError().c_str());
		return 1;
	}

	const uint64_t single_us = bench_single(session, entries, rounds);
	const uint64_t batch_us = bench_batch(session, entries, rounds);
	if (single_us == 0 || batch_us == 0)
	{
		std::fprintf(stderr, "benchmark failed: %s\n", session.lastError().c_str());
		return 2;
	}

	std::printf("vars=%zu rounds=%u\n", entries.size(), rounds);
	std::printf("single: %llu us (%.1f us/sample)\n", static_cast<unsigned long long>(single_us),
		static_cast<double>(single_us) / (rounds * entries.size()));
	std::printf("batch:  %llu us (%.1f us/sample)\n", static_cast<unsigned long long>(batch_us),
		static_cast<double>(batch_us) / rounds);
	std::printf("speedup: %.2fx\n", static_cast<double>(single_us) / static_cast<double>(batch_us));
	return 0;
}
