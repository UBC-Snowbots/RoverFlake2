// morse_code.h — morse table + encode/decode helpers
//
// Table mirrors rover_vision/morse_decoder_node.py — keep the two in sync.

#pragma once

#include <QMap>
#include <QString>
#include <QStringList>

namespace rover_hmi_core::morse {

inline const QMap<QString, QChar>& table() {
    static const QMap<QString, QChar> t = {
        {".-", 'a'},    {"-...", 'b'},  {"-.-.", 'c'},  {"-..", 'd'},   {".", 'e'},
        {"..-.", 'f'},  {"--.", 'g'},   {"....", 'h'},  {"..", 'i'},    {".---", 'j'},
        {"-.-", 'k'},   {".-..", 'l'},  {"--", 'm'},    {"-.", 'n'},    {"---", 'o'},
        {".--.", 'p'},  {"--.-", 'q'},  {".-.", 'r'},   {"...", 's'},   {"-", 't'},
        {"..-", 'u'},   {"...-", 'v'},  {".--", 'w'},   {"-..-", 'x'},  {"-.--", 'y'},
        {"--..", 'z'},  {"-----", '0'}, {".----", '1'}, {"..---", '2'}, {"...--", '3'},
        {"....-", '4'}, {".....", '5'}, {"-....", '6'}, {"--...", '7'}, {"---..", '8'},
        {"----.", '9'}, {".-.-.-", '.'}, {"--..--", ','}, {"---...", ':'}, {"..--..", '?'},
        {".----.", '\''}, {"-....-", '-'}, {"-..-.", '/'}, {"-.--.", '('}, {"-.--.-", ')'},
        {".-..-.", '"'}, {".--.-.", '@'}, {"-...-", '='},
    };
    return t;
}

// ". -- / .-" → "EM A"  ("/" separates words, "?" for unknown groups)
inline QString decodeGroups(QString in) {
    in.replace("/", " / ");
    QString out;
    for (const QString& g : in.split(' ', Qt::SkipEmptyParts)) {
        if (g == "/") { out += ' '; continue; }
        out += table().value(g, QChar('?'));
    }
    return out.toUpper();
}

// "hi u" → ".... ..  /  ..-"
inline QString encodeText(const QString& in) {
    static const QMap<QChar, QString> rev = [] {
        QMap<QChar, QString> r;
        for (auto it = table().cbegin(); it != table().cend(); ++it) r[it.value()] = it.key();
        return r;
    }();
    QStringList words;
    for (const QString& w : in.toLower().split(' ', Qt::SkipEmptyParts)) {
        QStringList groups;
        for (QChar c : w) groups << rev.value(c, "?");
        words << groups.join("  ");
    }
    return words.join("   /   ");
}

}  // namespace rover_hmi_core::morse
