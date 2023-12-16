# Configuring the Robot

Setup networking through the GUI:

    IPv4 Method: Manual
    IP: 10.42.0.1
    Netmask: 255.255.255.0
    Check for "Use this connection only for resources on its network"


Run the playbook:

    ansible-playbook ubr_ros1.yml -i inventory --ask-become-pass
    ansible-playbook ubr_humble.yml -i inventory --ask-become-pass
    ansible-playbook ubr_iron.yml -i inventory --ask-become-pass
