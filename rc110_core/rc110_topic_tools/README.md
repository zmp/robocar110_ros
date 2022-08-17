# Namespace Relay Node

The node allows to change the current ros namespace to a custom one for a set of topics.

## Subscribed Topics
```
~input_ns [std_msgs::String]
    input namespace (empty by default)
    
~output_ns [std_msgs::String]
    output namespace (empty by default)
    
{/input_ns/}<topicN> [<any>::<any>]
    input topics
```

## Published Topics
```
{/output_ns/}<topicN> [<any>::<any>]
    relayed topics
```

## Parameters
```    
~topics (vector, default: [])
    input topic names
```
