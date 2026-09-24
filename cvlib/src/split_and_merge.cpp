/* Split and merge segmentation algorithm implementation.
 * @file
 * @date 2018-09-05
 * @author Anonymous
 */
/*
#include "cvlib.hpp"

namespace
{
void split_image(cv::Mat image, double stddev)
{
    cv::Mat mean;
    cv::Mat dev;
    cv::meanStdDev(image, mean, dev);

    if (dev.at<double>(0) <= stddev)
    {
        image.setTo(mean);
        return;
    }

    const auto width = image.cols;
    const auto height = image.rows;

    split_image(image(cv::Range(0, height / 2), cv::Range(0, width / 2)), stddev);
    split_image(image(cv::Range(0, height / 2), cv::Range(width / 2, width)), stddev);
    split_image(image(cv::Range(height / 2, height), cv::Range(width / 2, width)), stddev);
    split_image(image(cv::Range(height / 2, height), cv::Range(0, width / 2)), stddev);
}
} // namespace

namespace cvlib
{
cv::Mat split_and_merge(const cv::Mat& image, double stddev)
{
    // split part
    cv::Mat res = image;
    split_image(res, stddev);

    // merge part
    // \todo implement merge algorithm
    return res;
}
} // namespace cvlib
*/

#include "cvlib.hpp"

#include <algorithm>
#include <map>
#include <vector>
//Объявляю структуру чтобы хранить информацию о наших областях, а так же нужные нам функции и классы
namespace
{
struct Region
{
    cv::Rect rect;
    double mean;
};

void split_image(const cv::Mat& image, double stddev, std::vector<Region>& regions, int offsetX = 0, int offsetY = 0)
{
    cv::Mat mean, dev;
    cv::meanStdDev(image, mean, dev);
    double currentStd = dev.at<double>(0);

    // Однородный регион или минимальный размер
    if (currentStd <= stddev || image.cols <= 1 || image.rows <= 1)
    {
        Region r;
        r.rect = cv::Rect(offsetX, offsetY, image.cols, image.rows);
        r.mean = mean.at<double>(0);
        regions.push_back(r);
        return;
    }

    int halfW = image.cols / 2;
    int halfH = image.rows / 2;
    //сплитуем по сути как и было
    split_image(image(cv::Range(0, halfH), cv::Range(0, halfW)), stddev, regions, offsetX, offsetY);
    split_image(image(cv::Range(0, halfH), cv::Range(halfW, image.cols)), stddev, regions, offsetX + halfW, offsetY);
    split_image(image(cv::Range(halfH, image.rows), cv::Range(0, halfW)), stddev, regions, offsetX, offsetY + halfH);
    split_image(image(cv::Range(halfH, image.rows), cv::Range(halfW, image.cols)), stddev, regions, offsetX + halfW, offsetY + halfH);
}
//проверяю есть ли общая граница
bool areNeighbors(const cv::Rect& a, const cv::Rect& b)
{
    if (a.x + a.width == b.x || b.x + b.width == a.x)
    {
        int y1 = std::max(a.y, b.y);
        int y2 = std::min(a.y + a.height, b.y + b.height);
        if (y2 - y1 > 0)
            return true;
    }
    if (a.y + a.height == b.y || b.y + b.height == a.y)
    {
        int x1 = std::max(a.x, b.x);
        int x2 = std::min(a.x + a.width, b.x + b.width);
        if (x2 - x1 > 0)
            return true;
    }
    return false;
}
//система непересекающихся множеств
class DSU
{
    public:
    std::vector<int> parent, rank_;
    explicit DSU(int n) : parent(n), rank_(n, 0)
    {
        for (int i = 0; i < n; ++i)
            parent[i] = i;
    }
    int find(int x)
    {
        return parent[x] == x ? x : parent[x] = find(parent[x]);
    }
    void unite(int a, int b)
    {
        a = find(a);
        b = find(b);
        if (a == b)
            return;
        if (rank_[a] < rank_[b])
            std::swap(a, b);
        parent[b] = a;
        if (rank_[a] == rank_[b])
            ++rank_[a];
    }
};
} // namespace

namespace cvlib
{
cv::Mat split_and_merge(const cv::Mat& image, double stddev)
{
    if (image.empty())
    {
        return cv::Mat();
    }

    cv::Mat gray;
    if (image.channels() == 3)
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else
        gray = image;

    // --- SPLIT ---
    std::vector<Region> regions;
    split_image(gray, stddev, regions);

    int n = static_cast<int>(regions.size());
    if (n == 0)
    {
        return image.clone();
    }

    // --- MERGE ---
    //закидываем все получившиеся регионы в dsu
    DSU dsu(n);

    std::map<double, std::vector<int>> byMean;
    for (int i = 0; i < n; ++i)
    {
        //закидываем всё в мапу чтобы быстро искать кандидатов на слияние
        byMean[regions[i].mean].push_back(i);
    }

    // Порог слияния: регионы сливаются, только если их средние равны
    // (для целочисленных изображений разница < 1).
    //следует из тестов
    const double mergeThreshold = 0.5;

    for (int i = 0; i < n; ++i)
    {
        //если не корень
        if (dsu.find(i) != i)
            continue;

        double lo = regions[i].mean - mergeThreshold;
        double hi = regions[i].mean + mergeThreshold;
        //границы по мапе
        auto itLo = byMean.lower_bound(lo);
        auto itHi = byMean.upper_bound(hi);
        //проверяем соседство всех найденных
        for (auto it = itLo; it != itHi; ++it)
        {
            for (int j : it->second)
            {
                if (j == i)
                    continue;
                if (dsu.find(j) == dsu.find(i))
                    continue;
                if (!areNeighbors(regions[i].rect, regions[j].rect))
                    continue;

                dsu.unite(i, j);
            }
        }
    }

    // --- Отрисовка ---
    cv::Mat res = image.clone();

    // Собираем корни и их средние
    std::map<int, double> rootMean;
    std::map<int, cv::Rect> rootRect;

    for (int i = 0; i < n; ++i)
    {
        int root = dsu.find(i);
        if (rootMean.find(root) == rootMean.end())
        {
            rootMean[root] = regions[i].mean;
            rootRect[root] = regions[i].rect;
        }
        else
        {
            // Взвешенное среднее по площади
            double areaOld = static_cast<double>(rootRect[root].area());
            double areaNew = static_cast<double>(regions[i].rect.area());
            rootMean[root] = (rootMean[root] * areaOld + regions[i].mean * areaNew) / (areaOld + areaNew);
            rootRect[root] = rootRect[root] | regions[i].rect;
        }
    }

    for (auto& kv : rootRect)
    {
        cv::Rect r = kv.second;
        r &= cv::Rect(0, 0, res.cols, res.rows);
        if (r.width <= 0 || r.height <= 0)
            continue;

        cv::Mat roi = res(r);
        if (image.channels() == 3)
        {
            roi.setTo(cv::Scalar(rootMean[kv.first], rootMean[kv.first], rootMean[kv.first]));
        }
        else
        {
            roi.setTo(cv::Scalar(rootMean[kv.first]));
        }
    }

    return res;
}
} // namespace cvlib