#include "robot_monitor/map_file_loader.h"
#include <QDir>
#include <QFileInfo>
#include <QImage>
#include <QString>

#include <fstream>
#include <sstream>

#include <jsoncpp/json/json.h>
#include <yaml-cpp/yaml.h>

namespace robot_monitor
{

MapFileLoader::MapFileLoader()
{
}

MapFileLoader::~MapFileLoader()
{
}

std::string MapFileLoader::resolveImagePath(const std::string& yaml_path,
                                            const std::string& image_field) const
{
    QFileInfo yaml_info(QString::fromStdString(yaml_path));
    QFileInfo image_info(QString::fromStdString(image_field));

    if (image_info.isAbsolute())
    {
        return image_field;
    }

    return QFileInfo(yaml_info.dir(), QString::fromStdString(image_field)).absoluteFilePath().toStdString();
}

bool MapFileLoader::loadRosMapFromYaml(const std::string& yaml_path,
                                       GridMapData& map_data,
                                       std::string& error_message)
{
    map_data = GridMapData();

    try
    {
        YAML::Node root = YAML::LoadFile(yaml_path);

        if (!root["image"] || !root["resolution"] || !root["origin"])
        {
            error_message = "YAML is missing required fields: image / resolution / origin";
            return false;
        }

        const std::string image_field = root["image"].as<std::string>();
        const double resolution = root["resolution"].as<double>();

        const YAML::Node origin = root["origin"];
        if (!origin.IsSequence() || origin.size() < 2)
        {
            error_message = "YAML field 'origin' is invalid";
            return false;
        }

        const double origin_x = origin[0].as<double>();
        const double origin_y = origin[1].as<double>();

        const double occupied_thresh = root["occupied_thresh"] ? root["occupied_thresh"].as<double>() : 0.65;
        const double free_thresh = root["free_thresh"] ? root["free_thresh"].as<double>() : 0.196;
        const int negate = root["negate"] ? root["negate"].as<int>() : 0;

        const std::string image_path = resolveImagePath(yaml_path, image_field);

        QImage image(QString::fromStdString(image_path));
        if (image.isNull())
        {
            error_message = "Failed to load map image: " + image_path;
            return false;
        }

        QImage gray = image.convertToFormat(QImage::Format_Grayscale8);

        map_data.width = gray.width();
        map_data.height = gray.height();
        map_data.resolution = resolution;
        map_data.origin_x = origin_x;
        map_data.origin_y = origin_y;
        map_data.data.resize(map_data.width * map_data.height);

        for (int y = 0; y < map_data.height; ++y)
        {
            for (int x = 0; x < map_data.width; ++x)
            {
                const int idx = y * map_data.width + x;
                const int pixel = qGray(gray.pixel(x, y));

                double occ = (255.0 - pixel) / 255.0;
                if (negate)
                {
                    occ = 1.0 - occ;
                }

                int8_t value = -1;
                if (occ > occupied_thresh)
                {
                    value = 100;
                }
                else if (occ < free_thresh)
                {
                    value = 0;
                }
                else
                {
                    value = -1;
                }

                // 存回 GridMapData，用原始 map 坐标顺序（不在这里翻转）
                const int store_y = map_data.height - 1 - y;
                const int store_idx = store_y * map_data.width + x;
                map_data.data[store_idx] = value;
            }
        }

        map_data.valid = true;
        return true;
    }
    catch (const YAML::Exception& e)
    {
        error_message = std::string("YAML parse error: ") + e.what();
        return false;
    }
    catch (const std::exception& e)
    {
        error_message = std::string("Map load error: ") + e.what();
        return false;
    }
}

bool MapFileLoader::loadImageMapFromPng(const std::string& image_path,
                                        GridMapData& map_data,
                                        std::string& error_message)
{
    map_data = GridMapData();

    QImage image(QString::fromStdString(image_path));
    if (image.isNull())
    {
        error_message = "Failed to load PNG image: " + image_path;
        return false;
    }

    QImage gray = image.convertToFormat(QImage::Format_Grayscale8);

    map_data.width = gray.width();
    map_data.height = gray.height();
    map_data.resolution = 1.0;
    map_data.origin_x = 0.0;
    map_data.origin_y = 0.0;
    map_data.data.resize(map_data.width * map_data.height);

    for (int y = 0; y < map_data.height; ++y)
    {
        for (int x = 0; x < map_data.width; ++x)
        {
            const int pixel = qGray(gray.pixel(x, y));

            int8_t value = -1;
            if (pixel < 100)
            {
                value = 100;   // occupied
            }
            else if (pixel > 220)
            {
                value = 0;     // free
            }
            else
            {
                value = -1;    // unknown
            }

            const int store_y = map_data.height - 1 - y;
            const int store_idx = store_y * map_data.width + x;
            map_data.data[store_idx] = value;
        }
    }

    map_data.valid = true;
    return true;
}

}  // namespace robot_monitor