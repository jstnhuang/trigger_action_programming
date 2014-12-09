#!/usr/bin/env python
import node_factory

def main():
    main_node = node_factory.build('trigger_action_programming')
    main_node.start()

if __name__ == '__main__':
    main()
