/*
 * test_list_remove.cpp
 *
 *  Created on: Nov 19, 2016
 *      Author: jsola
 */

#include <memory>
#include <list>
#include <algorithm>
#include <iostream>

int main()
{
    using std::list;
    using std::shared_ptr;
    using std::make_shared;
    using std::cout;
    using std::endl;

    typedef shared_ptr<int> IntPtr;

    list<IntPtr> L;

    L.push_back(make_shared<int>(0));
    L.push_back(make_shared<int>(1));
    L.push_back(make_shared<int>(2));

    cout << "size: " << L.size() << endl;

//    for (auto p : L)
//    {
//        cout << "removing " << *p << endl;
//        L.remove(p);
//        cout << "size: " << L.size() << endl;
//    }

//    list<IntPtr>::iterator it = L.begin();
//    while (it != L.end())
//    {
//        cout << "removing " << **it << endl;
//        L.erase(it);
//        it = L.begin();
//        cout << "size: " << L.size() << endl;
//    }

    list<IntPtr> L_black;
    for (auto p : L)
    {
        cout << "marking " << *p << endl;
        L_black.push_back(p);
        cout << "size: " << L.size() << endl;
    }
    for (auto p : L_black)
    {
        cout << "removing " << *p << endl;
        L.remove(p);
        cout << "size: " << L.size() << endl;
    }
    return 0;
}
