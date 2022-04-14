# Namespace Relay Node

The node allows to change the current ros namespace to a custom one for a set of topics.

## Subscribed Topics
```   
{/input_ns_param/}<topicN> [<any>::<any>]
    input topics
```

## Published Topics
```
{/output_ns_param/}<topicN> [<any>::<any>]
    relayed topics
```

## Parameters
```
input_ns_param [string, default: ""]
    parameter which contains input namespace
    
output_ns_param [string, default: ""]
    parameter which contains output namespace
    
topics (vector, default: [])
    input topic names
```
