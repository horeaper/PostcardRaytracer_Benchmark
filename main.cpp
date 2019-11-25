#include "stdafx.h"

struct Vec
{
	float x, y, z;

	Vec(float v = 0) { x = y = z = v; }

	Vec(float a, float b, float c = 0)
	{
		x = a;
		y = b;
		z = c;
	}

	Vec operator + (Vec r) const { return { x + r.x, y + r.y, z + r.z }; }
	Vec operator * (Vec r) const { return { x * r.x, y * r.y, z * r.z }; }
	float operator % (Vec r) const { return x * r.x + y * r.y + z * r.z; }

	// intnv square root
	Vec operator ! () const { return *this * (1 / sqrtf(*this % *this)); }
};

float randomVal() { return (float)rand() / RAND_MAX; }

// Rectangle CSG equation. Returns minimum signed distance from
// space carved by
// lowerLeft vertex and opposite rectangle vertex upperRight.
float BoxTest(Vec position, Vec lowerLeft, Vec upperRight)
{
	lowerLeft = position + lowerLeft * -1;
	upperRight = upperRight + position * -1;
	return -min(min(min(lowerLeft.x, upperRight.x), min(lowerLeft.y, upperRight.y)), min(lowerLeft.z, upperRight.z));
}

constexpr int HIT_NONE = 0;
constexpr int HIT_LETTER = 1;
constexpr int HIT_WALL = 2;
constexpr int HIT_SUN = 3;

const char letters[] = // 15 two points lines
	"5O5_" "5W9W" "5_9_"			// P (without curve)
	"AOEO" "COC_" "A_E_"			// I
	"IOQ_" "I_QO"					// X
	"UOY_" "Y_]O" "WW[W"			// A
	"aOa_" "aWeW" "a_e_" "cWiO";	// R (without curve)

// Sample the world using Signed Distance Fields.
float QueryDatabase(Vec position, int &hitType)
{
	float distance = 1e9;
	Vec f = position; // Flattened position (z=0)
	f.z = 0;

	for (int i = 0; i < 60; i += 4) {
		Vec begin = Vec(letters[i] - 79.0f, letters[i + 1] - 79.0f) * 0.5f;
		Vec e = Vec(letters[i + 2] - 79.0f, letters[i + 3] - 79.0f) * 0.5f + begin * -1;
		Vec o = f + (begin + e * min(-min((begin + f * -1) % e / (e % e), 0), 1)) * -1;
		distance = min(distance, o % o); // compare squared distance.
	}
	distance = sqrtf(distance); // Get real distance, not square distance.

	// Two curves (for P and R in PixaR) with hard-coded locations.
	Vec curves[] = { Vec(-11, 6), Vec(11, 6) };
	for (int i = 2; i--;) {
		Vec o = f + curves[i] * -1;
		distance = min(distance, o.x > 0 ? fabsf(sqrtf(o % o) - 2) : (o.y += o.y > 0 ? -2 : 2, sqrtf(o % o)));
	}
	distance = powf(powf(distance, 8) + powf(position.z, 8), 0.125f) - 0.5f;
	hitType = HIT_LETTER;

	float roomDist;
	roomDist = min( // min(A,B) = Union with Constructive solid geometry
		//-min carves an empty space
		-min( // Lower room
			BoxTest(position, Vec(-30, -.5, -30), Vec(30, 18, 30)),
			// Upper room
			BoxTest(position, Vec(-25, 17, -25), Vec(25, 20, 25))
		),
		BoxTest( // Ceiling "planks" spaced 8 units apart.
			Vec(fmodf(fabsf(position.x), 8), position.y, position.z),
			Vec(1.5, 18.5, -25),
			Vec(6.5, 20, 25)
		)
	);
	if (roomDist < distance) {
		distance = roomDist, hitType = HIT_WALL;
	}

	float sun = 19.9f - position.y; // Everything above 19.9 is light source.
	if (sun < distance) {
		distance = sun, hitType = HIT_SUN;
	}

	return distance;
}

// Perform signed sphere marching
// Returns hitType 0, 1, 2, or 3 and update hit position/normal
int RayMarching(Vec origin, Vec direction, Vec &hitPos, Vec &hitNorm)
{
	int hitType = HIT_NONE;
	int noHitCount = 0;
	float d; // distance from closest object in world.

	// Signed distance marching
	for (float total_d = 0; total_d < 100; total_d += d)
		if ((d = QueryDatabase(hitPos = origin + direction * total_d, hitType)) < 0.01f || ++noHitCount > 99)
			return hitNorm =
				   !Vec(QueryDatabase(hitPos + Vec(0.01f, 0), noHitCount) - d,
						QueryDatabase(hitPos + Vec(0, 0.01f), noHitCount) - d,
						QueryDatabase(hitPos + Vec(0, 0, 0.01f), noHitCount) - d)
				   , hitType; // Weird return statement where a variable is also updated.
	return 0;
}

Vec Trace(Vec origin, Vec direction)
{
	Vec sampledPosition, normal, color, attenuation = 1;
	Vec lightDirection(!Vec(0.6f, 0.6f, 1)); // Directional light

	for (int bounceCount = 0; bounceCount < 3;  ++bounceCount) {
		int hitType = RayMarching(origin, direction, sampledPosition, normal);
		if (hitType == HIT_NONE) {
			break; // No hit. This is over, return color.
		}
		if (hitType == HIT_LETTER) {
			// Specular bounce on a letter. No color acc.
			direction = direction + normal * (normal % direction * -2);
			origin = sampledPosition + direction * 0.1f;
			attenuation = attenuation * 0.2f; // Attenuation via distance traveled.
		}
		if (hitType == HIT_WALL) {
			// Wall hit uses color yellow?
			float incidence = normal % lightDirection;
			float p = 6.283185f * randomVal();
			float c = randomVal();
			float s = sqrtf(1 - c);
			float g = normal.z < 0 ? -1.0f : 1.0f;
			float u = -1 / (g + normal.z);
			float v = normal.x * normal.y * u;
			direction = Vec(v, g + normal.y * normal.y * u, -normal.y) * (cosf(p) * s)
						+
						Vec(1 + g * normal.x * normal.x * u, g * v, -g * normal.x) * (sinf(p) * s) + normal * sqrtf(c);
			origin = sampledPosition + direction * 0.1f;
			attenuation = attenuation * 0.2f;
			if (incidence > 0 && RayMarching(sampledPosition + normal * 0.1f, lightDirection, sampledPosition, normal) == HIT_SUN) {
				color = color + attenuation * Vec(500, 400, 100) * incidence;
			}
		}
		if (hitType == HIT_SUN) {
			color = color + attenuation * Vec(50, 80, 100);
			break; // Sun Color
		}
	}
	return color;
}

int main()
{
	const int width = 960;
	const int height = 540;

	const Vec position(-22, 5, 25);
	const Vec goal = !(Vec(-3, 4, 0) + position * -1);
	const Vec left = !Vec(goal.z, 0, -goal.x) * (1.0f / width);

	// Cross-product to get the up vector
	const Vec up(goal.y * left.z - goal.z * left.y,
				 goal.z * left.x - goal.x * left.z,
				 goal.x * left.y - goal.y * left.x);

	struct ColorValue
	{		
		uint8_t R;
		uint8_t G;
		uint8_t B;
	};

// 	ColorValue* ColorOutput = new ColorValue[width * height];

	auto fnProcessScanline = [&](int samples, int y) {
		for (int x = 0; x < width; ++x) {
			Vec color;
			for (int p = samples; p--;) {
				color = color + Trace(position, !(goal + left * (x - width / 2 + randomVal()) + up * (y - height / 2 + randomVal())));
			}

			// Reinhard tone mapping
			color = color * (1.0f / samples) + 14.0f / 241;
			const Vec o = color + 1;
			color = Vec(color.x / o.x, color.y / o.y, color.z / o.z) * 255;

/*
			auto &output = ColorOutput[x + y * width];
			output.R = (uint8_t)color.x;
			output.G = (uint8_t)color.y;
			output.B = (uint8_t)color.z;
*/
		}
	};

	SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);

	LARGE_INTEGER frequency;
	LARGE_INTEGER multi_begin, multi_end, single_begin, single_end;
	QueryPerformanceFrequency(&frequency);

	printf("Benchmarking single-threaded with 16 samples...");
	int samples = 16;
	QueryPerformanceCounter(&single_begin);
	for (int y = 0; y < height; ++y) {
		fnProcessScanline(samples, y);
	}
	QueryPerformanceCounter(&single_end);
	printf("\n");

	printf("Benchmarking multi-threaded with 128 samples...");
	samples = 128;
	QueryPerformanceCounter(&multi_begin);
	concurrency::parallel_for(0, height, [&](int y) { fnProcessScanline(samples, y); });
	QueryPerformanceCounter(&multi_end);
	printf("\n");

/*
	printf("P6 %d %d 255 ", width, height);
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			auto &color = ColorOutput[(width - x - 1) + (height - y - 1) * width];
			printf("%c%c%c", color.R, color.G, color.B);
		}
	}
	delete[] ColorOutput;
*/

	auto singleCounter = single_end.QuadPart - single_begin.QuadPart;
	auto multiCounter = multi_end.QuadPart - multi_begin.QuadPart;
	int singleTime = (int)((double)singleCounter / (double)frequency.QuadPart);
	int multiTime = (int)((double)multiCounter / (double)frequency.QuadPart);
	printf("Single Threaded: %dm%ds\n", singleTime / 60, singleTime % 60);
	printf(" Multi Threaded: %dm%ds\n", multiTime / 60, multiTime % 60);
	system("pause");
	
	return 0;
}
