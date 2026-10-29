#pragma once
// Small, strict JSON reader for the stdio protocol. No coercion of missing,
// null, string or malformed values into numeric measurements.
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

struct Json
{
    enum Type { Null, Number, String, Array, Object, Boolean } type = Null;
    double number = 0;
    std::string text;
    std::vector<Json> array;
    std::map<std::string, Json> object;
    const Json& get(const std::string& key) const
    {
        static const Json missing;
        auto it = object.find(key);
        return it == object.end() ? missing : it->second;
    }
    double numeric() const
    {
        if (type != Number) throw std::runtime_error("expected finite JSON number");
        return number;
    }
};

class JsonReader
{
    const std::string& s;
    size_t p = 0;
    [[noreturn]] void bad() const { throw std::runtime_error("malformed JSON request"); }
    void ws() { while (p < s.size() && std::isspace(static_cast<unsigned char>(s[p]))) ++p; }
    bool take(char c) { ws(); if (p < s.size() && s[p] == c) { ++p; return true; } return false; }
    unsigned hex4()
    {
        unsigned n = 0;
        for (int i=0; i<4; ++i)
        {
            if (p == s.size()) bad();
            char c=s[p++];
            n <<= 4;
            if (c>='0' && c<='9') n += c-'0';
            else if (c>='a' && c<='f') n += c-'a'+10;
            else if (c>='A' && c<='F') n += c-'A'+10;
            else bad();
        }
        return n;
    }
    static void utf8(std::string& out, unsigned n)
    {
        if (n < 0x80) out += char(n);
        else if (n < 0x800) { out += char(0xc0|(n>>6)); out += char(0x80|(n&63)); }
        else if (n < 0x10000) { out += char(0xe0|(n>>12)); out += char(0x80|((n>>6)&63)); out += char(0x80|(n&63)); }
        else { out += char(0xf0|(n>>18)); out += char(0x80|((n>>12)&63)); out += char(0x80|((n>>6)&63)); out += char(0x80|(n&63)); }
    }
    std::string string()
    {
        if (!take('"')) bad();
        std::string out;
        while (p < s.size())
        {
            unsigned char c=s[p++];
            if (c == '"') return out;
            if (c < 32) bad();
            if (c != '\\') { out += char(c); continue; }
            if (p == s.size()) bad();
            c=s[p++];
            switch(c)
            {
                case '"': case '\\': case '/': out += char(c); break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {
                    unsigned n=hex4();
                    if (n>=0xd800 && n<=0xdbff)
                    {
                        if (p+2>s.size() || s[p++]!='\\' || s[p++]!='u') bad();
                        unsigned low=hex4();
                        if (low<0xdc00 || low>0xdfff) bad();
                        n=0x10000+((n-0xd800)<<10)+(low-0xdc00);
                    }
                    else if (n>=0xdc00 && n<=0xdfff) bad();
                    utf8(out,n); break;
                }
                default: bad();
            }
        }
        bad();
    }
    Json value(unsigned depth)
    {
        if (depth > 32) bad();
        ws();
        if (p == s.size()) bad();
        Json j;
        if (s[p]=='"') { j.type=Json::String; j.text=string(); return j; }
        if (take('{'))
        {
            j.type=Json::Object;
            if (take('}')) return j;
            do { auto k=string(); if (!take(':')) bad();
                 if (!j.object.emplace(k, value(depth+1)).second) bad();
            } while (take(','));
            if (!take('}')) bad();
            return j;
        }
        if (take('['))
        {
            j.type=Json::Array;
            if (take(']')) return j;
            do { j.array.push_back(value(depth+1)); } while (take(','));
            if (!take(']')) bad();
            return j;
        }
        for (const char* literal : {"null", "true", "false"})
        {
            const std::string t=literal;
            if (s.compare(p,t.size(),t)==0)
            { p+=t.size(); j.type=t=="null" ? Json::Null : Json::Boolean; j.number=t=="true"; return j; }
        }
        const size_t start=p;
        if (s[p]=='-') ++p;
        if (p==s.size()) bad();
        if (s[p]=='0') ++p;
        else { if (s[p]<'1'||s[p]>'9') bad(); while(p<s.size()&&std::isdigit(s[p])) ++p; }
        if (p<s.size() && s[p]=='.')
        { ++p; size_t b=p; while(p<s.size()&&std::isdigit(s[p])) ++p; if(p==b) bad(); }
        if (p<s.size() && (s[p]=='e'||s[p]=='E'))
        { ++p; if(p<s.size()&&(s[p]=='+'||s[p]=='-')) ++p; size_t b=p;
          while(p<s.size()&&std::isdigit(s[p])) ++p;
          if(p==b) bad(); }
        j.type=Json::Number;
        j.number=std::strtod(s.substr(start,p-start).c_str(),nullptr);
        if (!std::isfinite(j.number)) bad();
        return j;
    }
public:
    explicit JsonReader(const std::string& input) : s(input) {}
    Json parse()
    {
        Json j=value(0); ws();
        if (p != s.size() || j.type != Json::Object) bad();
        return j;
    }
};
