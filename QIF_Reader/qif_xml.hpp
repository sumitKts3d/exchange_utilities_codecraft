////////////////////////////////////////////////////////////////////////////////
/// Copyright (c) 2010 - 2026 by Tech Soft 3D, Inc.
///
/// The information contained herein is confidential and proprietary to
/// Tech Soft 3D, Inc., and considered a trade secret as defined under civil
/// and criminal statutes. Tech Soft 3D shall pursue its civil and criminal
/// remedies in the event of unauthorized use or misappropriation of its trade
/// secrets. Use of this information by anyone other than authorized employees
/// of Tech Soft 3D, Inc. is granted only under a written non-disclosure
/// agreement, expressly prescribing the scope and manner of such use.
////////////////////////////////////////////////////////////////////////////////

// qif_xml.hpp - minimal, dependency-free XML DOM reader for QIF 3.0 instance files.
//
// Handles: XML declaration / processing instructions, comments, DOCTYPE (skipped),
// CDATA, attributes in ' or ", the five predefined entities and numeric character
// references. Namespace prefixes are stripped from element and attribute names
// (QIF uses one default namespace, http://qifstandards.org/xsd/qif3).
// Not a validating parser: it is meant for well-formed QIF files.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace qifxml {

struct Node
{
    std::string name;                                        // local name, prefix stripped
    std::vector<std::pair<std::string, std::string>> attrs;  // local names
    std::vector<std::unique_ptr<Node>> children;
    std::string text;                                        // concatenated character data
    Node* parent = nullptr;

    const Node* child(const char* n) const
    {
        for (const auto& c : children)
            if (c->name == n) return c.get();
        return nullptr;
    }
    const char* attr(const char* n) const
    {
        for (const auto& a : attrs)
            if (a.first == n) return a.second.c_str();
        return nullptr;
    }
    bool attrBool(const char* n, bool def = false) const
    {
        const char* v = attr(n);
        if (!v) return def;
        std::string s(v);
        return s == "true" || s == "1";
    }
    // Text of a direct child element, or "" if absent.
    std::string childText(const char* n) const
    {
        const Node* c = child(n);
        return c ? c->text : std::string();
    }
};

class ParseError : public std::runtime_error
{
public:
    ParseError(const std::string& msg, size_t line) : std::runtime_error(msg + " (line " + std::to_string(line) + ")") {}
};

class Parser
{
public:
    explicit Parser(const std::string& data) : s_(data) {}

    std::unique_ptr<Node> parse()
    {
        // Skip UTF-8 BOM.
        if (s_.size() >= 3 && (unsigned char)s_[0] == 0xEF && (unsigned char)s_[1] == 0xBB && (unsigned char)s_[2] == 0xBF) p_ = 3;
        std::unique_ptr<Node> root;
        while (p_ < s_.size())
        {
            skipWs();
            if (p_ >= s_.size()) break;
            if (s_[p_] != '<') fail("unexpected character data outside the root element");
            if (startsWith("<?")) { skipPast("?>"); continue; }
            if (startsWith("<!--")) { skipPast("-->"); continue; }
            if (startsWith("<!DOCTYPE")) { skipDoctype(); continue; }
            if (root) fail("more than one root element");
            root = parseElement(nullptr);
        }
        if (!root) fail("no root element");
        return root;
    }

private:
    const std::string& s_;
    size_t p_ = 0;

    [[noreturn]] void fail(const std::string& m) const
    {
        size_t line = 1;
        for (size_t i = 0; i < p_ && i < s_.size(); ++i) if (s_[i] == '\n') ++line;
        throw ParseError("XML: " + m, line);
    }
    bool startsWith(const char* t) const { return s_.compare(p_, std::char_traits<char>::length(t), t) == 0; }
    static bool isWs(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }
    void skipWs() { while (p_ < s_.size() && isWs(s_[p_])) ++p_; }
    void skipPast(const char* t)
    {
        size_t q = s_.find(t, p_);
        if (q == std::string::npos) fail(std::string("unterminated construct, expected ") + t);
        p_ = q + std::char_traits<char>::length(t);
    }
    void skipDoctype()
    {
        int depth = 0;
        for (; p_ < s_.size(); ++p_)
        {
            if (s_[p_] == '[') ++depth;
            else if (s_[p_] == ']') --depth;
            else if (s_[p_] == '>' && depth <= 0) { ++p_; return; }
        }
        fail("unterminated DOCTYPE");
    }
    static std::string local(const std::string& n)
    {
        size_t c = n.find(':');
        return c == std::string::npos ? n : n.substr(c + 1);
    }
    std::string readName()
    {
        size_t b = p_;
        while (p_ < s_.size() && !isWs(s_[p_]) && s_[p_] != '>' && s_[p_] != '/' && s_[p_] != '=') ++p_;
        if (b == p_) fail("expected a name");
        return s_.substr(b, p_ - b);
    }
    static void appendUtf8(std::string& out, uint32_t cp)
    {
        if (cp < 0x80) out += (char)cp;
        else if (cp < 0x800) { out += (char)(0xC0 | (cp >> 6)); out += (char)(0x80 | (cp & 0x3F)); }
        else if (cp < 0x10000) { out += (char)(0xE0 | (cp >> 12)); out += (char)(0x80 | ((cp >> 6) & 0x3F)); out += (char)(0x80 | (cp & 0x3F)); }
        else { out += (char)(0xF0 | (cp >> 18)); out += (char)(0x80 | ((cp >> 12) & 0x3F)); out += (char)(0x80 | ((cp >> 6) & 0x3F)); out += (char)(0x80 | (cp & 0x3F)); }
    }
    // Decode entities in s_[b, e) and append to out.
    void decode(size_t b, size_t e, std::string& out) const
    {
        for (size_t i = b; i < e; ++i)
        {
            char c = s_[i];
            if (c != '&') { out += c; continue; }
            size_t semi = s_.find(';', i);
            if (semi == std::string::npos || semi >= e) { out += c; continue; }
            std::string ent = s_.substr(i + 1, semi - i - 1);
            if (ent == "lt") out += '<';
            else if (ent == "gt") out += '>';
            else if (ent == "amp") out += '&';
            else if (ent == "quot") out += '"';
            else if (ent == "apos") out += '\'';
            else if (!ent.empty() && ent[0] == '#')
            {
                uint32_t cp = (ent.size() > 1 && (ent[1] == 'x' || ent[1] == 'X'))
                                  ? (uint32_t)std::stoul(ent.substr(2), nullptr, 16)
                                  : (uint32_t)std::stoul(ent.substr(1), nullptr, 10);
                appendUtf8(out, cp);
            }
            else { out.append(s_, i, semi - i + 1); }
            i = semi;
        }
    }
    std::unique_ptr<Node> parseElement(Node* parent)
    {
        // at '<'
        ++p_;
        auto node = std::make_unique<Node>();
        node->parent = parent;
        node->name = local(readName());
        for (;;)
        {
            skipWs();
            if (p_ >= s_.size()) fail("unterminated start tag <" + node->name + ">");
            if (s_[p_] == '/')
            {
                if (p_ + 1 >= s_.size() || s_[p_ + 1] != '>') fail("expected '/>'");
                p_ += 2;
                return node;
            }
            if (s_[p_] == '>') { ++p_; break; }
            std::string an = readName();
            skipWs();
            if (p_ >= s_.size() || s_[p_] != '=') fail("expected '=' after attribute " + an);
            ++p_;
            skipWs();
            if (p_ >= s_.size() || (s_[p_] != '"' && s_[p_] != '\'')) fail("expected quoted attribute value");
            char q = s_[p_++];
            size_t e = s_.find(q, p_);
            if (e == std::string::npos) fail("unterminated attribute value");
            std::string v;
            decode(p_, e, v);
            p_ = e + 1;
            // xmlns declarations are not data
            if (an == "xmlns" || an.compare(0, 6, "xmlns:") == 0) continue;
            node->attrs.emplace_back(local(an), std::move(v));
        }
        // content
        for (;;)
        {
            if (p_ >= s_.size()) fail("unterminated element <" + node->name + ">");
            if (s_[p_] == '<')
            {
                if (startsWith("</"))
                {
                    p_ += 2;
                    std::string en = local(readName());
                    skipWs();
                    if (p_ >= s_.size() || s_[p_] != '>') fail("expected '>' in end tag");
                    ++p_;
                    if (en != node->name) fail("mismatched end tag </" + en + "> for <" + node->name + ">");
                    return node;
                }
                if (startsWith("<!--")) { skipPast("-->"); continue; }
                if (startsWith("<![CDATA["))
                {
                    p_ += 9;
                    size_t e = s_.find("]]>", p_);
                    if (e == std::string::npos) fail("unterminated CDATA");
                    node->text.append(s_, p_, e - p_);
                    p_ = e + 3;
                    continue;
                }
                if (startsWith("<?")) { skipPast("?>"); continue; }
                node->children.push_back(parseElement(node.get()));
            }
            else
            {
                size_t e = s_.find('<', p_);
                if (e == std::string::npos) e = s_.size();
                decode(p_, e, node->text);
                p_ = e;
            }
        }
    }
};

inline std::unique_ptr<Node> parseDocument(const std::string& data)
{
    Parser p(data);
    return p.parse();
}

} // namespace qifxml
